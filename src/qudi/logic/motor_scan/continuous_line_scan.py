# -*- coding: utf-8 -*-
"""
Continuous line scanning mixin for MotorScanLogic.

Handles continuous line-by-line scanning for CONTINUOUS_STREAM and CONTINUOUS_FREQ_TRACK modes,
where the motor moves continuously along each line without stopping at individual grid points.
"""

import time
from typing import Dict, Optional

from PySide2 import QtCore

from .data_structures import ScanMode, ScanState


class ContinuousLineScanMixin:
    """
    Mixin class providing continuous line scanning functionality for MotorScanLogic.
    
    This mixin provides:
    - Line-by-line continuous motor movement (no stopping at grid points within a line)
    - Position sampling during movement
    - Data binning after each line completes
    - Handling of pause/resume mid-line
    
    The parent class must provide:
    - _motor_hardware: Connector to motor hardware
    - _scan_data: MotorScanData instance
    - _thread_lock: RecursiveMutex for thread safety
    - _scan_state, _stop_requested: State variables
    - _position_poll_interval: ConfigOption
    - _start_position_sampling, _stop_position_sampling: From MotorControlMixin
    - _bin_line_data, _get_line_raw_data_buffer, _clear_line_raw_data_buffer: From DataProcessingMixin
    - sigScanDataUpdated, sigScanStateChanged: Signals
    - log: Logger instance
    """

    # Position tolerance for verifying motor arrived at target (meters)
    _LINE_START_POSITION_TOLERANCE = 500e-6  # 500 µm
    # Maximum retries for re-issuing move command when motor reports idle at wrong position
    _LINE_START_MAX_RETRIES = 3
    # Timeout for reaching line start position (seconds). Long moves (e.g., 20mm)
    # can take 10-30s on KDC101 stages. The timeout covers the entire move, not
    # individual retries.
    _LINE_START_TIMEOUT = 120.0

    def _init_continuous_line_state(self):
        """Initialize state variables for continuous line scanning. Call from __init__."""
        self._continuous_line_mode = True  # Enable continuous line scanning
        self._current_line_index = 0
        self._line_data_start_time = 0.0
        self._line_motor_poll_timer = None
        self._waiting_for_line_start = False
        self._waiting_for_line_end = False
        self._line_start_position = {}  # Target start position for current line
        self._line_end_position = {}  # Target end position for current line
        self._line_paused_mid_scan = False
        self._line_pause_position = None  # Position where pause occurred
        self._line_pause_position_buffer = []  # Preserved position samples during pause
        self._line_start_retries = 0  # Retry counter for reaching line start

    def _should_use_continuous_line_mode(self) -> bool:
        """
        Check if continuous line mode should be used for current scan.
        
        Returns:
            True if scan mode supports continuous movement and it's enabled.
        """
        if self._scan_data is None:
            return False
        
        # Only use for CONTINUOUS_* modes, POSITION_ONLY, and KDC_HW_SYNC*
        if self._scan_data.scan_mode not in (ScanMode.CONTINUOUS_STREAM, ScanMode.CONTINUOUS_FREQ_TRACK,
                                             ScanMode.POSITION_ONLY, ScanMode.KDC_HW_SYNC,
                                             ScanMode.KDC_HW_SYNC_MULTIRES):
            return False
        
        # Check if continuous line mode is enabled
        return getattr(self, '_continuous_line_mode_enabled', True)

    def _start_continuous_line_scan(self, line_index: int):
        """
        Start scanning a single line continuously.
        
        Moves to line start, then initiates continuous movement to line end
        while recording positions.
        
        Args:
            line_index: Zero-based index of the line to scan.
        """
        with self._thread_lock:
            if self._scan_data is None:
                self.log.error("No scan data available")
                return
            
            self._current_line_index = line_index
            if self._scan_data.scan_mode in (ScanMode.KDC_HW_SYNC, ScanMode.KDC_HW_SYNC_MULTIRES):
                # Fast-axis endpoints over-travel half a bin past the outer bin
                # boundaries so the stage crosses every boundary (all ppl+1 pulses).
                start_pos, end_pos = self._hw_sync_line_endpoints(line_index)
            else:
                start_pos, end_pos = self._scan_data.get_line_start_end_positions(line_index)

            if not start_pos or not end_pos:
                self.log.error(f"Could not get positions for line {line_index}")
                self._finalize_scan(completed=False)
                return
            
            n_lines = self._scan_data.get_num_lines()
            points_per_line = self._scan_data.get_points_per_line()

            # Log positions for debugging direction issues
            start_str = ', '.join(f'{k}={v*1000:.3f}mm' for k, v in start_pos.items())
            end_str = ', '.join(f'{k}={v*1000:.3f}mm' for k, v in end_pos.items())
            self.log.info(f"Starting continuous line {line_index + 1}/{n_lines} "
                         f"({points_per_line} points): start=({start_str}), end=({end_str})")
            
            # Move to line start position (blocking)
            motor = self._motor_hardware()
            if motor is None:
                self.log.error("Motor hardware not available")
                return
            
            # Start by moving to line start
            result = motor.move_abs(start_pos)
            if result != 0:
                self.log.error(
                    f"Failed to issue move to line {line_index} start position: {start_pos}"
                )
                self._finalize_scan(completed=False)
                return
            self.sigScanStatusMessage.emit(f'Moving to line {line_index + 1}/{n_lines} start...')

            # Wait for arrival at line start
            self._waiting_for_line_start = True
            self._line_start_position = start_pos
            self._line_end_position = end_pos
            self._line_start_retries = 0
            self._line_start_move_time = time.time()

            # Initialize line motor poll timer if needed
            if self._line_motor_poll_timer is None:
                self._line_motor_poll_timer = QtCore.QTimer()
                self._line_motor_poll_timer.setSingleShot(True)
                self._line_motor_poll_timer.timeout.connect(
                    self._on_line_motor_poll_timeout,
                    QtCore.Qt.QueuedConnection
                )

            # Use longer initial delay (200ms) to let KDC101 process the USB
            # move command before we start polling is_moving(). Without this,
            # the first poll can see is_moving()=False because the motor hasn't
            # started yet, causing the scan to proceed from the wrong position.
            self._line_motor_poll_timer.start(200)

    @QtCore.Slot()
    def _on_line_motor_poll_timeout(self):
        """Called periodically during continuous line scanning."""
        with self._thread_lock:
            if self._stop_requested or self._scan_state == ScanState.STOPPING:
                self._stop_position_sampling()
                motor = self._motor_hardware()
                if motor is not None:
                    motor.abort()
                self._finalize_scan(completed=False)
                return
            
            if self._scan_state == ScanState.PAUSED:
                # Pause mid-line: stop motor, record position, stop sampling but preserve data
                if not self._line_paused_mid_scan:
                    motor = self._motor_hardware()
                    if motor is not None:
                        motor.abort()
                        self._line_pause_position = motor.get_pos()
                    if self._scan_data.scan_mode in (ScanMode.KDC_HW_SYNC, ScanMode.KDC_HW_SYNC_MULTIRES):
                        # Continuous hardware-marker capture can't be cleanly resumed
                        # mid-line; re-scan the whole current line on resume (its grid
                        # row is overwritten when the line completes).
                        self._hw_line_restart_on_resume = True
                    else:
                        # Preserve position samples collected so far for resume
                        self._line_pause_position_buffer = self._stop_position_sampling()
                    self._line_paused_mid_scan = True
                    self.log.info(f"Line {self._current_line_index} paused mid-scan at position: "
                                 f"{self._line_pause_position}")
                return
            
            motor = self._motor_hardware()
            if motor is None:
                return
            
            is_moving = motor.is_moving() if hasattr(motor, 'is_moving') else False

            # KDC_HW_SYNC: drain the FPGA demod + marker rings every poll, in EVERY
            # phase (including inter-line / move-to-start), so the rings never sit
            # undrained during a long move. The push client NaN-fills any gap and
            # preserves the absolute sample count, so the demod array index stays
            # exactly aligned with the FPGA marker sample indices regardless of when
            # we drain.
            if self._scan_data.scan_mode in (ScanMode.KDC_HW_SYNC, ScanMode.KDC_HW_SYNC_MULTIRES):
                self._hw_sync_drain()

            if hasattr(self, '_waiting_for_line_start') and self._waiting_for_line_start:
                # Phase 1: Waiting to arrive at line start
                if is_moving:
                    self._line_motor_poll_timer.start(50)
                    return

                # Motor reports not moving — verify we actually reached the
                # target start position on ALL axes. The KDC101 can report
                # not-moving if:
                # (a) the USB move command hasn't been processed yet (race), or
                # (b) the stage genuinely stopped short of the target.
                # We must check all axes (not just the fast axis) because the
                # slow axis also needs to reach its target before the line scan
                # begins — otherwise a line scan at y=20mm could run at y=0.
                actual_pos = motor.get_pos()
                max_position_error = 0.0
                worst_axis = ''
                for axis, target in self._line_start_position.items():
                    error = abs(actual_pos.get(axis, 0) - target)
                    if error > max_position_error:
                        max_position_error = error
                        worst_axis = axis

                if max_position_error > self._LINE_START_POSITION_TOLERANCE:
                    elapsed = time.time() - self._line_start_move_time

                    # Check overall timeout first
                    if elapsed > self._LINE_START_TIMEOUT:
                        self.log.error(
                            f"Line {self._current_line_index}: timed out reaching start "
                            f"position after {elapsed:.1f}s "
                            f"({worst_axis} error={max_position_error*1e6:.0f}µm). "
                            f"Aborting scan."
                        )
                        self._finalize_scan(completed=False)
                        return

                    # Motor says idle but position is wrong. Re-issue the move
                    # command (handles USB race where command wasn't processed).
                    # Limit re-issues to avoid flooding USB, but keep polling
                    # with a longer interval to allow the motor time to move.
                    self._line_start_retries += 1
                    if self._line_start_retries <= self._LINE_START_MAX_RETRIES:
                        target_val = self._line_start_position.get(worst_axis, 0)
                        actual_val = actual_pos.get(worst_axis, 0)
                        self.log.warning(
                            f"Line {self._current_line_index}: motor not at start position "
                            f"({worst_axis}: target={target_val*1000:.3f}mm, "
                            f"actual={actual_val*1000:.3f}mm, "
                            f"error={max_position_error*1e6:.0f}µm). "
                            f"Re-issuing move (attempt {self._line_start_retries}/"
                            f"{self._LINE_START_MAX_RETRIES})."
                        )
                        result = motor.move_abs(self._line_start_position)
                        if result != 0:
                            self.log.error(
                                f"Line {self._current_line_index}: failed to re-issue "
                                f"move to start position. Aborting scan."
                            )
                            self._finalize_scan(completed=False)
                            return
                    # Keep polling — motor may need time to reach target.
                    # Use 500ms interval to avoid flooding USB with get_pos() calls.
                    self._line_motor_poll_timer.start(500)
                    return

                # Position verified — log actual position for traceability
                pos_str = ', '.join(
                    f'{ax}={actual_pos.get(ax, 0)*1000:.3f}mm' for ax in self._line_start_position
                )
                if self._line_start_retries > 0:
                    self.log.info(
                        f"Line {self._current_line_index}: start position reached "
                        f"after {self._line_start_retries} re-issue(s): ({pos_str}), "
                        f"max error={max_position_error*1e6:.0f}µm"
                    )
                else:
                    self.log.debug(
                        f"Line {self._current_line_index}: start position verified: "
                        f"({pos_str}), max error={max_position_error*1e6:.0f}µm"
                    )

                self._waiting_for_line_start = False
                self._waiting_for_line_end = True

                self._line_data_start_time = time.time()
                if self._scan_data.scan_mode in (ScanMode.KDC_HW_SYNC, ScanMode.KDC_HW_SYNC_MULTIRES):
                    # Hardware data path: configure this line's fast-axis trigger and
                    # reset the per-line x-marker baseline. No software sampling.
                    self._hw_sync_line_start(self._current_line_index)
                else:
                    # Clear data buffers and start software position sampling
                    self._clear_line_raw_data_buffer()
                    sample_interval_ms = int(getattr(self, '_position_poll_interval', 0.05) * 1000)
                    self._start_position_sampling(sample_interval_ms)

                # Issue move to line end — ONLY command the fast axis.
                # Sending both axes causes a race condition: for LINE_BY_LINE_Y,
                # move_abs({'x': current, 'y': target}) sends the x no-op first
                # (dict iteration order), then y second. The poll can fire in the
                # gap after x's no-op completes but before y starts, seeing both
                # axes idle and concluding the line is "complete" at y=0.
                # The slow axis is already at the correct position from Phase 1.
                fast_axis = self._scan_data.get_fast_axis()
                fast_axis_move = {fast_axis: self._line_end_position[fast_axis]}
                result = motor.move_abs(fast_axis_move)
                if result != 0:
                    self.log.error(
                        f"Line {self._current_line_index}: failed to issue move "
                        f"to line end position: {fast_axis_move}"
                    )
                    self._stop_position_sampling()
                    self._finalize_scan(completed=False)
                    return

                n_lines = self._scan_data.get_num_lines()
                self.log.info(f"Scanning line {self._current_line_index + 1}/{n_lines} "
                              f"({fast_axis}: {self._line_start_position[fast_axis]*1000:.3f}"
                              f" -> {self._line_end_position[fast_axis]*1000:.3f}mm)")
                self.sigScanStatusMessage.emit(
                    f'Scanning line {self._current_line_index + 1}/{n_lines}...'
                )
                # Use 200ms initial delay to let KDC101 process the USB command
                self._line_motor_poll_timer.start(200)
                
            elif self._waiting_for_line_end:
                # Phase 2: Waiting to arrive at line end (continuous scan in progress).
                # Draining is handled once at the top of this handler (all phases).
                if is_moving:
                    self._line_motor_poll_timer.start(50)
                    return
                
                # Line complete - stop sampling and bin data
                self._waiting_for_line_end = False

                if self._scan_data.scan_mode in (ScanMode.KDC_HW_SYNC, ScanMode.KDC_HW_SYNC_MULTIRES):
                    # Hardware data path: final drain + reconstruct this line's grid
                    # row from the x-markers captured during the sweep.
                    self._hw_sync_line_finish(self._current_line_index)
                elif self._scan_data.scan_mode == ScanMode.POSITION_ONLY:
                    self._stop_position_sampling()
                    self.log.debug(f"POSITION_ONLY: Line {self._current_line_index} completed (no data)")
                else:
                    position_buffer = self._stop_position_sampling()
                    raw_data_buffer = self._get_line_raw_data_buffer()

                    # Bin the collected data
                    bin_result = self._bin_line_data(
                        line_index=self._current_line_index,
                        position_time_buffer=position_buffer,
                        raw_data_buffer=raw_data_buffer,
                        data_start_time=self._line_data_start_time
                    )

                    if not bin_result.get('success', False):
                        self.log.warning(f"Binning failed for line {self._current_line_index}: "
                                       f"{bin_result.get('error', 'unknown')}")
                        # Clean up failed line data to prevent corruption of next line
                        self._clear_line_raw_data_buffer()
                
                # Validate motor reached line end (Finding #4: motor stop detection)
                fast_axis = self._scan_data.get_fast_axis()
                expected_end = self._line_end_position.get(fast_axis, 0)
                final_pos = motor.get_pos()
                actual_end = final_pos.get(fast_axis, 0)
                distance_from_target = abs(expected_end - actual_end)
                
                if distance_from_target > 1e-3:  # 1mm threshold
                    self.log.warning(
                        f"Line {self._current_line_index} stopped short: "
                        f"expected {expected_end*1000:.2f}mm, actual {actual_end*1000:.2f}mm. "
                        f"Data for this line may be degraded."
                    )
                
                # Update point index to reflect all points in this line
                points_in_line = self._scan_data.get_points_per_line()
                self._scan_data.current_point_index = (self._current_line_index + 1) * points_in_line
                
                # Emit data update for GUI
                self.sigScanDataUpdated.emit()
                
                # Check if scan is complete
                next_line = self._current_line_index + 1
                if next_line >= self._scan_data.get_num_lines():
                    self._finalize_scan(completed=True)
                else:
                    # Start next line
                    self._start_continuous_line_scan(next_line)

    def _resume_continuous_line(self):
        """
        Resume a paused continuous line scan.
        
        If paused mid-line, completes the current line from pause position,
        preserving position samples collected before pause.
        """
        with self._thread_lock:
            if not self._line_paused_mid_scan:
                # Normal resume - start next line
                return False

            self._line_paused_mid_scan = False

            if self._scan_data.scan_mode in (ScanMode.KDC_HW_SYNC, ScanMode.KDC_HW_SYNC_MULTIRES) and self._hw_line_restart_on_resume:
                # Re-scan the whole current line from its start (markers/demod keep
                # streaming; the per-line baseline reset in _hw_sync_line_start
                # discards the abandoned partial line's markers).
                self._hw_line_restart_on_resume = False
                self.log.info(f"Resuming KDC_HW_SYNC by re-scanning line {self._current_line_index}")
                self._start_continuous_line_scan(self._current_line_index)
                return True
            
            # Resume from pause position to line end
            self.log.info(f"Resuming line {self._current_line_index} from pause position")
            
            # Restore pre-pause position samples into the buffer, then start
            # sampling with preserve_buffer=True so _start_position_sampling()
            # does not clear them.
            if hasattr(self, '_line_pause_position_buffer') and self._line_pause_position_buffer:
                self._position_sample_buffer = self._line_pause_position_buffer.copy()
                self._line_pause_position_buffer = []
            else:
                self._position_sample_buffer = []

            sample_interval_ms = int(getattr(self, '_position_sample_interval', 0.05) * 1000)
            self._start_position_sampling(sample_interval_ms, preserve_buffer=True)
            
            # Continue movement to line end
            motor = self._motor_hardware()
            if motor is not None:
                fast_axis = self._scan_data.get_fast_axis()
                fast_axis_move = {fast_axis: self._line_end_position[fast_axis]}
                result = motor.move_abs(fast_axis_move)
                if result != 0:
                    self.log.error(
                        f"Line {self._current_line_index}: failed to resume move "
                        f"to line end position: {fast_axis_move}"
                    )
                    self._stop_position_sampling()
                    self._finalize_scan(completed=False)
                    return False
            
            self._waiting_for_line_end = True
            self._line_motor_poll_timer.start(200)
            
            return True
