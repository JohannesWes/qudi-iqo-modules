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
                motor.move_abs_sync(position)
            else:
                motor.move_abs(position)
                if hasattr(motor, 'wait_for_idle'):
                    motor.wait_for_idle()

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
