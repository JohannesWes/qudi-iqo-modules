# -*- coding: utf-8 -*-
"""
Motor control mixin for MotorScanLogic.

Handles motor movement, homing, and position polling functionality.
"""

import time
from typing import Dict, List

from PySide2 import QtCore


class MotorControlMixin:
    """
    Mixin class providing motor control functionality for MotorScanLogic.
    
    This mixin provides:
    - Manual motor movement (blocking and non-blocking)
    - Stage homing/calibration
    - Position polling and update signals
    
    The parent class must provide:
    - _motor_hardware: Connector to motor hardware
    - _thread_lock: RecursiveMutex for thread safety
    - is_scanning: Property indicating if scan is in progress
    - _homing_in_progress: bool flag
    - _moving_in_progress: bool flag
    - sigMovementStateChanged: Signal(bool)
    - sigHomingStateChanged: Signal(bool)
    - sigPositionUpdated: Signal(dict)
    - _sigDoMove: Signal(dict)
    - _sigDoHoming: Signal(object)
    - log: Logger instance
    """

    @QtCore.Slot(dict)
    def move_to_position(self, position: Dict[str, float]):
        """
        Move motors to specified position (non-blocking).

        This method returns immediately. The actual movement runs asynchronously
        on the logic thread. Emits sigMovementStateChanged(True) when starting
        and sigMovementStateChanged(False) when complete.

        Args:
            position: Dict mapping axis name to target position (in meters).
        """
        with self._thread_lock:
            if self.is_scanning:
                self.log.warning("Cannot move during scan.")
                return

            if self._homing_in_progress:
                self.log.warning("Cannot move while homing is in progress.")
                return

            if self._moving_in_progress:
                self.log.warning("Movement already in progress.")
                return

            motor = self._motor_hardware()
            if motor is None:
                self.log.error("Motor hardware not available.")
                return

            self.log.info(f"Moving to position: {position}")
            self._moving_in_progress = True
            self.sigMovementStateChanged.emit(True)

            # Emit signal to run move asynchronously on logic thread
            self._sigDoMove.emit(position)

    @QtCore.Slot(dict)
    def _do_move_async(self, position: Dict[str, float]):
        """
        Perform the actual move operation (runs on logic thread via signal).

        This method is invoked via _sigDoMove with QueuedConnection to ensure
        it runs on the logic's thread, not blocking the GUI thread.

        Args:
            position: Dict mapping axis name to target position (in meters).
        """
        motor = self._motor_hardware()
        if motor is None:
            self.log.error("Motor hardware not available.")
            self._moving_in_progress = False
            self.sigMovementStateChanged.emit(False)
            return

        try:
            # Use synchronous move with position verification
            if hasattr(motor, 'move_abs_sync'):
                result = motor.move_abs_sync(position)
                if result != 0:
                    self.log.error(f"Move did not reach target position: {position}")
                    self.sigPositionUpdated.emit(self.current_position)
                    return
            else:
                result = motor.move_abs(position)
                if result != 0:
                    self.log.error(f"Failed to issue move command: {position}")
                    self.sigPositionUpdated.emit(self.current_position)
                    return
                if hasattr(motor, 'wait_for_idle'):
                    if not motor.wait_for_idle():
                        self.log.error(f"Move timed out before reaching target: {position}")
                        self.sigPositionUpdated.emit(self.current_position)
                        return

            # Read and verify actual position from hardware
            actual_pos = self.current_position
            if actual_pos:
                # Log achieved position
                pos_str = ', '.join(f'{k}={v*1000:.3f}mm' for k, v in actual_pos.items())
                self.log.info(f"Movement completed. Position: {pos_str}")

                # Check for position errors
                for axis, target in position.items():
                    if axis in actual_pos:
                        error = abs(actual_pos[axis] - target)
                        if error > 100e-6:  # 100 µm threshold for warning
                            self.log.warning(
                                f"{axis}-axis position error: target={target*1000:.3f}mm, "
                                f"actual={actual_pos[axis]*1000:.3f}mm, error={error*1e6:.1f}µm"
                            )
            else:
                self.log.info("Movement completed (position readback unavailable).")

            self.sigPositionUpdated.emit(actual_pos)

        except Exception as e:
            self.log.error(f"Move failed: {e}", exc_info=True)
        finally:
            self._moving_in_progress = False
            self.sigMovementStateChanged.emit(False)

    @QtCore.Slot()
    def stop_movement(self):
        """
        Stop current movement and reset state.

        Can be called to abort an in-progress move operation.
        """
        with self._thread_lock:
            if not self._moving_in_progress:
                return

            motor = self._motor_hardware()
            if motor is not None:
                motor.abort()

            self._moving_in_progress = False
            self.sigMovementStateChanged.emit(False)
            self.sigPositionUpdated.emit(self.current_position)
            self.log.info("Movement stopped by user.")
    
    @QtCore.Slot()
    def _on_position_poll_timeout(self):
        """Poll current position and emit update signal (called by timer)."""
        pos = self.current_position
        if pos:
            self.sigPositionUpdated.emit(pos)

    @QtCore.Slot()
    def home_stages(self, axes: List[str] = None):
        """
        Home (calibrate) the motor stages (non-blocking).
        
        Moves all stages to their home position and establishes zero reference.
        Cannot be called during a scan. This method returns immediately and
        emits sigHomingStateChanged(True) when starting and sigHomingStateChanged(False)
        when complete.
        
        Args:
            axes: Optional list of axes to home. If None, homes all available axes.
        """
        with self._thread_lock:
            if self.is_scanning:
                self.log.error("Cannot home stages during a scan.")
                return
            
            if self._homing_in_progress:
                self.log.warning("Homing already in progress.")
                return
            
            motor = self._motor_hardware()
            if motor is None:
                self.log.error("Motor hardware not available.")
                return
            
            if axes is None:
                axes = self.available_axes
            
            self.log.info(f"Homing stages: {axes}")
            self._homing_in_progress = True
            self._homing_axes = list(axes) if axes else []
            self.sigHomingStateChanged.emit(True)
            
            # Emit signal to run homing asynchronously on logic thread
            self._sigDoHoming.emit(self._homing_axes)

    @QtCore.Slot(object)
    def _do_homing_async(self, axes: List[str]):
        """
        Perform the actual homing operation (runs on logic thread via signal).
        
        This method is invoked via _sigDoHoming with QueuedConnection to ensure
        it runs on the logic's thread, not blocking the GUI thread.
        
        Args:
            axes: List of axes to home.
        """
        motor = self._motor_hardware()
        if motor is None:
            self.log.error("Motor hardware not available.")
            self._homing_in_progress = False
            self.sigHomingStateChanged.emit(False)
            return
        
        self._homing_start_time = time.time()
        
        try:
            result = motor.calibrate(list(axes) if axes else None)
            elapsed = time.time() - self._homing_start_time
            
            if result == 0:
                self.log.info(f"Homing completed in {elapsed:.1f}s")
            else:
                self.log.warning(f"Homing returned error code: {result}")
            
            self.sigPositionUpdated.emit(self.current_position)
            
        except Exception as e:
            self.log.error(f"Homing failed: {e}", exc_info=True)
        finally:
            self._homing_in_progress = False
            self._homing_axes = []
            self.sigHomingStateChanged.emit(False)

    @QtCore.Slot()
    def _on_homing_poll_timeout(self):
        """
        Poll homing status (placeholder for future async homing implementations).
        
        Note: Currently not used since motor.calibrate() blocks internally.
        This method is available for future use if hardware supports truly
        async homing where we can poll is_moving() separately.
        """
        pass

    # =========================================================================
    # Position Sampling for Continuous Line Scanning
    # =========================================================================

    def _init_position_sampling_state(self):
        """Initialize state variables for position sampling. Call from __init__."""
        self._position_sample_buffer = []  # List of (timestamp, {axis: position})
        self._position_sample_timer = None
        self._position_sampling_active = False
        self._line_scan_start_time = 0.0

    def _start_position_sampling(self, sample_interval_ms: int = 50,
                                  preserve_buffer: bool = False):
        """
        Start recording timestamped positions at regular intervals.

        Args:
            sample_interval_ms: Interval between position samples in milliseconds.
            preserve_buffer: If True, keep existing buffer contents (used when
                resuming a paused line scan to retain pre-pause position samples).
        """
        if not preserve_buffer:
            self._position_sample_buffer = []
            self._line_scan_start_time = time.time()
        self._position_sampling_active = True
        
        # Create timer if needed
        if self._position_sample_timer is None:
            self._position_sample_timer = QtCore.QTimer()
            self._position_sample_timer.timeout.connect(
                self._on_position_sample_timeout,
                QtCore.Qt.QueuedConnection
            )
        
        # Record initial position
        self._record_position_sample()
        
        # Start periodic sampling
        self._position_sample_timer.start(sample_interval_ms)
        self.log.debug(f"Position sampling started at {1000/sample_interval_ms:.0f} Hz")

    def _stop_position_sampling(self) -> list:
        """
        Stop position sampling and return the recorded buffer.
        
        Returns:
            List of (timestamp, {axis: position}) tuples recorded during sampling.
        """
        if self._position_sample_timer is not None:
            self._position_sample_timer.stop()
        
        # Record final position
        if self._position_sampling_active:
            self._record_position_sample()
        
        self._position_sampling_active = False
        buffer = self._position_sample_buffer.copy()
        
        if len(buffer) > 0:
            duration = buffer[-1][0] - buffer[0][0]
            self.log.debug(f"Position sampling stopped: {len(buffer)} samples over {duration:.2f}s")
        
        return buffer

    def _record_position_sample(self):
        """Record a single timestamped position sample."""
        try:
            timestamp = time.time() - self._line_scan_start_time
            position = self.current_position
            if position:
                # Detect identical consecutive position samples (expected at scan
                # endpoints when motor has stopped; could also indicate stale
                # encoder readback if it persists during movement).
                if len(self._position_sample_buffer) > 0:
                    _, prev_position = self._position_sample_buffer[-1]
                    if position == prev_position:
                        if not getattr(self, '_identical_pos_count', 0):
                            self._identical_pos_count = 1
                        else:
                            self._identical_pos_count += 1
                        # Only warn after many consecutive identical samples
                        # (suggests stale data during movement, not just stopped motor)
                        if self._identical_pos_count == 10:
                            self.log.warning(
                                "10+ identical consecutive position samples. "
                                "Motor may be returning stale encoder values."
                            )
                    else:
                        self._identical_pos_count = 0
                
                self._position_sample_buffer.append((timestamp, position.copy()))
        except Exception as e:
            self.log.debug(f"Failed to record position sample: {e}")

    @QtCore.Slot()
    def _on_position_sample_timeout(self):
        """Called periodically to record position samples."""
        if not self._position_sampling_active:
            return
        self._record_position_sample()
