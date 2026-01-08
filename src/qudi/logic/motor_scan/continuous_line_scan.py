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

    def _init_continuous_line_state(self):
        """Initialize state variables for continuous line scanning. Call from __init__."""
        self._continuous_line_mode = True  # Enable continuous line scanning
        self._current_line_index = 0
        self._line_data_start_time = 0.0
        self._line_motor_poll_timer = None
        self._waiting_for_line_start = False
        self._waiting_for_line_end = False
        self._line_end_position = {}  # Target end position for current line
        self._line_paused_mid_scan = False
        self._line_pause_position = None  # Position where pause occurred
        self._line_pause_position_buffer = []  # Preserved position samples during pause

    def _should_use_continuous_line_mode(self) -> bool:
        """
        Check if continuous line mode should be used for current scan.
        
        Returns:
            True if scan mode supports continuous movement and it's enabled.
        """
        if self._scan_data is None:
            return False
        
        # Only use for CONTINUOUS_* modes
        if self._scan_data.scan_mode not in (ScanMode.CONTINUOUS_STREAM, ScanMode.CONTINUOUS_FREQ_TRACK):
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
            start_pos, end_pos = self._scan_data.get_line_start_end_positions(line_index)
            
            if not start_pos or not end_pos:
                self.log.error(f"Could not get positions for line {line_index}")
                self._finalize_scan(completed=False)
                return
            
            n_lines = self._scan_data.get_num_lines()
            points_per_line = self._scan_data.get_points_per_line()
            
            self.log.info(f"Starting continuous line {line_index + 1}/{n_lines} "
                         f"({points_per_line} points)")
            
            # Move to line start position (blocking)
            motor = self._motor_hardware()
            if motor is None:
                self.log.error("Motor hardware not available")
                return
            
            # Start by moving to line start
            motor.move_abs(start_pos)
            
            # Wait for arrival at line start
            self._waiting_for_line_start = True
            self._line_end_position = end_pos
            
            # Initialize line motor poll timer if needed
            if self._line_motor_poll_timer is None:
                self._line_motor_poll_timer = QtCore.QTimer()
                self._line_motor_poll_timer.setSingleShot(True)
                self._line_motor_poll_timer.timeout.connect(
                    self._on_line_motor_poll_timeout,
                    QtCore.Qt.QueuedConnection
                )
            
            # Start polling for arrival at line start
            self._line_motor_poll_timer.start(50)

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
            
            if hasattr(self, '_waiting_for_line_start') and self._waiting_for_line_start:
                # Phase 1: Waiting to arrive at line start
                if is_moving:
                    self._line_motor_poll_timer.start(50)
                    return
                
                # Arrived at line start - begin continuous scan
                self._waiting_for_line_start = False
                self._waiting_for_line_end = True
                
                # Clear data buffers and start position sampling
                self._clear_line_raw_data_buffer()
                self._line_data_start_time = time.time()
                
                sample_interval_ms = int(getattr(self, '_position_poll_interval', 0.05) * 1000)
                self._start_position_sampling(sample_interval_ms)
                
                # Issue move to line end (non-blocking)
                motor.move_abs(self._line_end_position)
                
                self.log.debug(f"Line {self._current_line_index}: moving to end position")
                self._line_motor_poll_timer.start(50)
                
            elif self._waiting_for_line_end:
                # Phase 2: Waiting to arrive at line end (continuous scan in progress)
                if is_moving:
                    self._line_motor_poll_timer.start(50)
                    return
                
                # Line complete - stop sampling and bin data
                self._waiting_for_line_end = False
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
            
            # Resume from pause position to line end
            self.log.info(f"Resuming line {self._current_line_index} from pause position")
            
            # FIX Finding #1: Prepare buffer with preserved samples BEFORE starting timer
            # to avoid race condition where timer fires before buffer is restored
            preserved = []
            if hasattr(self, '_line_pause_position_buffer') and self._line_pause_position_buffer:
                preserved = self._line_pause_position_buffer.copy()
                self._line_pause_position_buffer = []
            
            # Initialize position sample buffer with preserved samples
            self._position_sample_buffer = preserved
            
            # Now start sampling (timer will append to our pre-initialized buffer)
            sample_interval_ms = int(getattr(self, '_position_sample_interval', 0.05) * 1000)
            self._start_position_sampling(sample_interval_ms)
            
            # Continue movement to line end
            motor = self._motor_hardware()
            if motor is not None:
                motor.move_abs(self._line_end_position)
            
            self._waiting_for_line_end = True
            self._line_motor_poll_timer.start(50)
            
            return True
