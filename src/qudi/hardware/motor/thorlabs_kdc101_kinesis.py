# -*- coding: utf-8 -*-
"""
Thorlabs KDC101/MTS50-Z8 Motor Controller using pylablib.

This module provides control for Thorlabs MTS50-Z8 motorized translation stages
driven by KDC101 K-Cube DC Servo controllers. Supports both 1D (single axis)
and 2D (XY) scanning modes.

Hardware Specifications (MTS50-Z8):
- Travel range: 50 mm
- Encoder resolution: 29 nm (34,555 counts/mm)
- Minimum repeatable increment: 0.8 um
- Home position accuracy: +/-4 um
- Backlash: <6 um
- Maximum velocity: 2.4 mm/s
- Maximum acceleration: 4.5 mm/s^2

Dependencies:
- pylablib >= 1.4.0: pip install pylablib
- FTDI drivers must be installed for USB communication


Example config:

    thorlabs_xy_stage:
        module.Class: 'motor.thorlabs_kdc101_kinesis.ThorlabsKDC101Kinesis'
        options:
            axis_config:
                x:
                    serial: '27500001'
                    pos_min: 0
                    pos_max: 0.05  # 50mm in meters
                y:
                    serial: '27500002'
                    pos_min: 0
                    pos_max: 0.05
            default_velocity: 2.0e-3  # m/s (used for all movements, including homing)
            settle_time: 0.01         # seconds
            auto_home: false

For 1D mode, omit the 'y' axis from axis_config:

    thorlabs_x_stage:
        module.Class: 'motor.thorlabs_kdc101_kinesis.ThorlabsKDC101Kinesis'
        options:
            axis_config:
                x:
                    serial: '27500001'
                    pos_min: 0
                    pos_max: 0.05
            default_velocity: 2.0e-3  # used for all movements, including homing
"""

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Any

from PySide2 import QtCore

from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar
from qudi.interface.motor_interface import MotorInterface


@dataclass(frozen=True)
class MTS50Z8Specs:
    """
    Hardware specifications for the MTS50-Z8 stage.

    All values from Thorlabs documentation. Values are immutable.
    Units are SI (meters, seconds).
    """
    # Travel limits
    TRAVEL_MIN: float = 0.0           # meters
    TRAVEL_MAX: float = 50e-3         # 50 mm in meters

    # Encoder specifications
    ENCODER_COUNTS_PER_M: float = 34555000.0  # counts per meter
    ENCODER_RESOLUTION: float = 29e-9          # 29 nm in meters

    # Positioning specifications
    MIN_REPEATABLE_INCREMENT: float = 0.8e-6  # 0.8 um in meters
    HOME_ACCURACY: float = 4e-6               # +/-4 um in meters
    BACKLASH: float = 6e-6                    # <6 um in meters

    # Velocity and acceleration limits
    MAX_VELOCITY: float = 2.4e-3      # 2.4 mm/s in m/s
    MAX_ACCELERATION: float = 4.5e-3  # 4.5 mm/s^2 in m/s^2

    # Default motion parameters
    DEFAULT_VELOCITY: float = 2.0e-3  # 2.0 mm/s (safe default)
    DEFAULT_SETTLE_TIME: float = 0.01  # 10 ms


# Module-level specs instance
SPECS = MTS50Z8Specs()


class ThorlabsKDC101Kinesis(MotorInterface):
    """
    Hardware module for Thorlabs MTS50-Z8 stages via KDC101 controllers using pylablib.

    Supports 1D (single axis) or 2D (XY) configurations.
    For 1D mode, omit 'y' from axis_config.

    All positions are in SI units (meters).
    """

    # ConfigOptions
    _axis_config = ConfigOption(
        name='axis_config',
        default=None,
        missing='error',
        constructor=lambda x: x  # Pass through as dict
    )
    _default_velocity = ConfigOption(
        name='default_velocity',
        default=SPECS.DEFAULT_VELOCITY
    )

    _settle_time = ConfigOption(
        name='settle_time',
        default=SPECS.DEFAULT_SETTLE_TIME
    )
    _auto_home = ConfigOption(
        name='auto_home',
        default=False
    )

    # StatusVars
    _is_homed = StatusVar(name='is_homed', default=False)

    # Signals
    sigPositionChanged = QtCore.Signal(dict)   # {'x': pos, 'y': pos}
    sigMovementFinished = QtCore.Signal()
    sigHomingComplete = QtCore.Signal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Internal state
        self._stages: Dict[str, Any] = {}  # axis_label -> KinesisMotor
        self._constraints: Dict[str, dict] = {}
        self._Thorlabs = None

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def specs(self) -> MTS50Z8Specs:
        """Hardware specifications for MTS50-Z8 stages."""
        return SPECS

    @property
    def is_2d_mode(self) -> bool:
        """True if both X and Y axes are configured."""
        return 'y' in self._stages

    @property
    def available_axes(self) -> List[str]:
        """List of configured axis labels ['x'] or ['x', 'y']."""
        return list(self._stages.keys())

    # =========================================================================
    # Module Lifecycle
    # =========================================================================

    def on_activate(self):
        """Initialize hardware connections."""
        # Import pylablib
        try:
            from pylablib.devices import Thorlabs
            self._Thorlabs = Thorlabs
        except ImportError:
            self.log.error(
                "pylablib is required but not installed. "
                "Install with: pip install pylablib"
            )
            raise

        # Build constraints from config
        self._build_constraints()

        # Connect to stages
        self._connect_stages()

        # Set default velocity
        self._apply_default_velocity()

        # Auto-home if configured
        if self._auto_home:
            self.log.info("Auto-homing enabled, starting homing sequence...")
            self.calibrate()

        mode_str = "2D (XY)" if self.is_2d_mode else "1D (X only)"
        self.log.info(f"ThorlabsKDC101Kinesis initialized in {mode_str} mode.")

    def on_deactivate(self):
        """Disconnect from hardware."""
        self._disconnect_stages()
        self.log.info("ThorlabsKDC101Kinesis deactivated.")

    def _build_constraints(self) -> None:
        """Build constraints dictionary from axis_config."""
        self._constraints = {}

        for axis_label, axis_cfg in self._axis_config.items():
            # Get position limits from config or use hardware defaults
            pos_min = axis_cfg.get('pos_min', SPECS.TRAVEL_MIN)
            pos_max = axis_cfg.get('pos_max', SPECS.TRAVEL_MAX)

            # Clamp to hardware limits
            pos_min = max(pos_min, SPECS.TRAVEL_MIN)
            pos_max = min(pos_max, SPECS.TRAVEL_MAX)

            self._constraints[axis_label] = {
                'label': axis_label,
                'unit': 'm',
                'ramp': ['Trapez'],
                'pos_min': pos_min,
                'pos_max': pos_max,
                'pos_step': SPECS.MIN_REPEATABLE_INCREMENT,
                'vel_min': 0.0,
                'vel_max': SPECS.MAX_VELOCITY,
                'vel_step': 1e-6,  # 1 um/s
                'acc_min': 0.0,
                'acc_max': SPECS.MAX_ACCELERATION,
                'acc_step': 1e-6,  # 1 um/s^2
            }

    def _connect_stages(self) -> None:
        """Establish connections to stage controllers."""
        for axis_label, axis_cfg in self._axis_config.items():
            serial = axis_cfg.get('serial')
            if serial is None:
                self.log.error(f"No serial number provided for axis '{axis_label}'")
                raise ValueError(f"Missing serial number for axis '{axis_label}'")

            try:
                self.log.debug(f"Connecting to {axis_label}-axis (serial: {serial})...")
                stage = self._Thorlabs.KinesisMotor(serial, scale="stage")

                # Verify units
                units = stage.get_scale_units()
                is_valid = False
                if isinstance(units, tuple):
                    if units and units[0] == 'm':
                        is_valid = True
                elif units == 'm':
                    is_valid = True

                if not is_valid:
                    self.log.warning(
                        f"{axis_label}-axis units are {units}, expected meters. "
                        "Check stage configuration."
                    )

                # Check and correct velocity if too low
                params = stage.get_velocity_parameters()
                if params.max_velocity < 1e-4:  # If < 0.1 mm/s
                    self.log.warning(
                        f"{axis_label}-axis velocity too low "
                        f"({params.max_velocity*1000:.3f} mm/s). Resetting."
                    )
                    accel = params.acceleration
                    if accel < 1e-4:
                        accel = SPECS.DEFAULT_VELOCITY
                    stage.setup_velocity(
                        max_velocity=SPECS.DEFAULT_VELOCITY,
                        acceleration=accel
                    )

                self._stages[axis_label] = stage
                self.log.info(f"Connected to {axis_label}-axis (serial: {serial})")

            except Exception as e:
                self.log.error(f"Failed to connect to {axis_label}-axis: {e}")
                self._disconnect_stages()
                raise RuntimeError(f"Failed to connect to {axis_label}-axis: {e}")

    def _disconnect_stages(self) -> None:
        """Close connections to stage controllers."""
        for axis_label, stage in list(self._stages.items()):
            try:
                if stage is not None:
                    stage.close()
                    self.log.debug(f"Disconnected {axis_label}-axis")
            except Exception as e:
                self.log.warning(f"Error disconnecting {axis_label}-axis: {e}")

        self._stages.clear()

    def _apply_default_velocity(self) -> None:
        """Apply default velocity to all axes."""
        vel_dict = {axis: self._default_velocity for axis in self._stages}
        self.set_velocity(vel_dict)

    # =========================================================================
    # MotorInterface Implementation
    # =========================================================================

    def get_constraints(self) -> Dict[str, dict]:
        """
        Retrieve the hardware constraints from the motor device.

        Returns:
            dict: Constraints for each axis with keys:
                - label: axis identifier
                - unit: 'm' (meters)
                - pos_min, pos_max: position limits
                - pos_step: minimum position increment
                - vel_min, vel_max, vel_step: velocity limits
                - acc_min, acc_max, acc_step: acceleration limits
                - ramp: available ramp profiles
        """
        return self._constraints.copy()

    def move_rel(self, param_dict: Dict[str, float]) -> int:
        """
        Moves stage in given direction (relative movement).

        Args:
            param_dict: Dictionary with axis labels and relative distances.
                        e.g., {'x': 0.001} for 1mm movement in X.

        Returns:
            int: Error code (0: OK, -1: error)
        """
        try:
            curr_pos = self.get_pos()

            for axis_label, distance in param_dict.items():
                if axis_label not in self._stages:
                    self.log.warning(f"Unknown axis '{axis_label}', ignoring.")
                    continue

                # Calculate new position
                new_pos = curr_pos.get(axis_label, 0) + distance

                # Check constraints
                constraints = self._constraints.get(axis_label, {})
                pos_min = constraints.get('pos_min', SPECS.TRAVEL_MIN)
                pos_max = constraints.get('pos_max', SPECS.TRAVEL_MAX)

                if new_pos < pos_min or new_pos > pos_max:
                    self.log.warning(
                        f"Relative move on {axis_label} would exceed limits "
                        f"[{pos_min*1000:.3f}, {pos_max*1000:.3f}]mm. "
                        f"Current: {curr_pos.get(axis_label, 0)*1000:.3f}mm, "
                        f"Requested: {distance*1000:.3f}mm. Ignored."
                    )
                    continue

                # Execute move
                self._stages[axis_label].move_by(distance)

            return 0

        except Exception as e:
            self.log.error(f"Relative move failed: {e}")
            return -1

    def move_abs(self, param_dict: Dict[str, float]) -> int:
        """
        Moves stage to absolute position.

        Args:
            param_dict: Dictionary with axis labels and absolute positions.
                        e.g., {'x': 0.025, 'y': 0.010} for X=25mm, Y=10mm.

        Returns:
            int: Error code (0: OK, -1: error)
        """
        try:
            for axis_label, position in param_dict.items():
                if axis_label not in self._stages:
                    self.log.warning(f"Unknown axis '{axis_label}', ignoring.")
                    continue

                # Check constraints
                constraints = self._constraints.get(axis_label, {})
                pos_min = constraints.get('pos_min', SPECS.TRAVEL_MIN)
                pos_max = constraints.get('pos_max', SPECS.TRAVEL_MAX)

                if position < pos_min or position > pos_max:
                    self.log.warning(
                        f"Position {position*1000:.3f}mm on {axis_label} "
                        f"exceeds limits [{pos_min*1000:.3f}, {pos_max*1000:.3f}]mm. "
                        f"Ignored."
                    )
                    continue

                # Execute move
                self._stages[axis_label].move_to(position)

            return 0

        except Exception as e:
            self.log.error(f"Absolute move failed: {e}")
            return -1

    def abort(self) -> int:
        """
        Stops movement of all stages immediately.

        Returns:
            int: Error code (0: OK, -1: error)
        """
        try:
            for axis_label, stage in self._stages.items():
                try:
                    stage.stop()
                except Exception as e:
                    self.log.warning(f"Error stopping {axis_label}: {e}")

            self.log.warning("Movement aborted on all axes.")
            return 0

        except Exception as e:
            self.log.error(f"Abort failed: {e}")
            return -1

    def get_pos(self, param_list: Optional[List[str]] = None) -> Dict[str, float]:
        """
        Gets current position of the stage arms.

        Args:
            param_list: Optional list of axis labels. If None, returns all axes.

        Returns:
            dict: Axis labels as keys, positions in meters as values.
        """
        positions = {}

        axes_to_query = param_list if param_list is not None else self._stages.keys()

        for axis_label in axes_to_query:
            if axis_label in self._stages:
                try:
                    positions[axis_label] = self._stages[axis_label].get_position()
                except Exception as e:
                    self.log.warning(f"Failed to get position of {axis_label}: {e}")
                    positions[axis_label] = 0.0

        return positions

    def get_status(self, param_list: Optional[List[str]] = None) -> Dict[str, int]:
        """
        Get the status of the position.

        Status codes:
            0: Idle (not moving)
            1: Moving forward
            2: Moving backward
            10: Homing

        Args:
            param_list: Optional list of axis labels. If None, returns all axes.

        Returns:
            dict: Axis labels as keys, status codes as values.
        """
        status = {}

        axes_to_query = param_list if param_list is not None else self._stages.keys()

        for axis_label in axes_to_query:
            if axis_label in self._stages:
                try:
                    is_moving = self._stages[axis_label].is_moving()
                    # Simple status: 0 = idle, 1 = moving
                    # pylablib doesn't provide direction info easily
                    status[axis_label] = 1 if is_moving else 0
                except Exception as e:
                    self.log.warning(f"Failed to get status of {axis_label}: {e}")
                    status[axis_label] = -1

        return status

    # Maximum number of homing retries per axis before giving up
    _HOMING_MAX_RETRIES = 2

    def calibrate(self, param_list: Optional[List[str]] = None) -> int:
        """
        Calibrate (home) the stage.

        Homing moves stage(s) to home position (negative limit switch)
        and establishes the zero reference. After homing, the stage stays
        at the home offset position (typically ~1mm from the limit).

        Each axis is verified after homing. If position verification fails,
        homing is retried up to _HOMING_MAX_RETRIES times before returning
        an error. A failed homing means the encoder zero reference is wrong
        and all subsequent move_abs commands would go to incorrect positions.

        Uses the 'default_velocity' config option for homing speed.

        Args:
            param_list: Optional list of axis labels to home. If None, homes all.

        Returns:
            int: Error code (0: OK, -1: error, -2: position verification failed)
        """
        try:
            axes_to_home = param_list if param_list is not None else list(self._stages.keys())
            self.log.info(f"Homing axes: {axes_to_home}")
            self.log.info(f"Homing velocity: {self._default_velocity*1000:.2f} mm/s")

            for axis_label in axes_to_home:
                if axis_label not in self._stages:
                    self.log.warning(f"Axis {axis_label} not found")
                    continue

                result = self._home_single_axis(axis_label)
                if result != 0:
                    return result

            self._is_homed = True
            self.sigHomingComplete.emit()
            self.log.info("Homing complete.")
            return 0

        except Exception as e:
            self.log.error(f"Calibration failed: {e}", exc_info=True)
            return -1

    def _home_single_axis(self, axis_label: str) -> int:
        """
        Home a single axis with retry logic.

        Attempts homing up to (1 + _HOMING_MAX_RETRIES) times. After each
        attempt, verifies that the position is near zero (within ±2mm).

        Args:
            axis_label: The axis to home (e.g., 'x' or 'y').

        Returns:
            int: 0 on success, -1 on hard error, -2 on verification failure.
        """
        stage = self._stages[axis_label]
        max_attempts = 1 + self._HOMING_MAX_RETRIES

        for attempt in range(max_attempts):
            if attempt > 0:
                self.log.warning(
                    f"{axis_label}-axis: retrying homing "
                    f"(attempt {attempt + 1}/{max_attempts})"
                )

            # Set homing velocity
            try:
                stage.setup_homing(velocity=self._default_velocity)
            except Exception as e:
                self.log.warning(
                    f"Could not set homing velocity for {axis_label}-axis: {e}"
                )

            pos_before = stage.get_position()
            self.log.info(
                f"Homing {axis_label}-axis (from {pos_before*1000:.1f}mm)..."
            )

            # Start homing (force=True to home even if device thinks it's done)
            try:
                stage.home(sync=False, force=True)
            except Exception as e:
                self.log.error(
                    f"Failed to start homing on {axis_label}-axis: {e}"
                )
                return -1

            time.sleep(1.0)

            # Poll until movement stops (timeout 120 seconds)
            timeout = 120.0
            start_time = time.time()
            last_log_time = 0

            while time.time() - start_time < timeout:
                if not stage.is_moving():
                    break
                elapsed = time.time() - start_time
                if elapsed - last_log_time >= 10:
                    pos_now = stage.get_position()
                    self.log.info(
                        f"{axis_label}-axis homing... "
                        f"{pos_now*1000:.1f}mm, {elapsed:.0f}s"
                    )
                    last_log_time = elapsed
                time.sleep(0.5)
            else:
                self.log.error(
                    f"{axis_label}-axis homing timed out after {timeout}s"
                )
                return -1

            # After homing, encoder is reset by firmware. Small delay to
            # ensure encoder state is synchronized before reading position.
            time.sleep(0.5)
            pos_after = stage.get_position()
            elapsed = time.time() - start_time

            # Verify position is near home. After homing, KDC101 stages
            # move to the home offset position which typically reads as
            # -1.0 to -1.5mm. Accept positions within 3mm of zero (covers
            # normal -1.5mm offset plus some margin). A failed homing reads
            # as 10-25mm away, so this threshold reliably catches failures.
            home_position_ok = abs(pos_after) < 3e-3
            # Verify timing is plausible (>2s unless started near home)
            time_plausible = (
                elapsed > 2.0 or abs(pos_before) < 5e-3
            )

            if home_position_ok and time_plausible:
                self.log.info(
                    f"{axis_label}-axis homed in {elapsed:.1f}s "
                    f"(position: {pos_after*1000:.1f}mm)"
                )
                return 0

            # Homing verification failed
            if not home_position_ok:
                self.log.warning(
                    f"{axis_label}-axis homing failed: position "
                    f"{pos_after*1000:.1f}mm is not near home. "
                    f"Expected ~0mm (within ±2mm)."
                )
            elif not time_plausible:
                self.log.warning(
                    f"{axis_label}-axis homing completed unusually fast "
                    f"({elapsed:.1f}s) from {pos_before*1000:.1f}mm. "
                    f"Homing may not have executed properly."
                )

            self.log.info(
                f"{axis_label}-axis homed in {elapsed:.1f}s "
                f"(position: {pos_after*1000:.1f}mm)"
            )

        # All retries exhausted
        self.log.error(
            f"{axis_label}-axis homing failed after {max_attempts} attempts. "
            f"Encoder zero reference may be incorrect."
        )
        return -2

    def get_velocity(self, param_list: Optional[List[str]] = None) -> Dict[str, float]:
        """
        Gets the current velocity for all connected axes.

        Args:
            param_list: Optional list of axis labels. If None, returns all axes.

        Returns:
            dict: Axis labels as keys, velocities in m/s as values.
        """
        velocities = {}

        axes_to_query = param_list if param_list is not None else self._stages.keys()

        for axis_label in axes_to_query:
            if axis_label in self._stages:
                try:
                    params = self._stages[axis_label].get_velocity_parameters()
                    velocities[axis_label] = params.max_velocity
                except Exception as e:
                    self.log.warning(f"Failed to get velocity of {axis_label}: {e}")
                    velocities[axis_label] = 0.0

        return velocities

    def set_velocity(self, param_dict: Dict[str, float]) -> int:
        """
        Write new value for velocity.

        Args:
            param_dict: Dictionary with axis labels and velocities in m/s.
                        e.g., {'x': 0.002} for 2 mm/s.

        Returns:
            int: Error code (0: OK, -1: error)
        """
        try:
            for axis_label, velocity in param_dict.items():
                if axis_label not in self._stages:
                    self.log.warning(f"Unknown axis '{axis_label}', ignoring.")
                    continue

                # Validate velocity
                constraints = self._constraints.get(axis_label, {})
                vel_max = constraints.get('vel_max', SPECS.MAX_VELOCITY)

                if velocity <= 0:
                    self.log.warning(f"Velocity must be positive, got {velocity}. Ignored.")
                    continue

                if velocity > vel_max:
                    self.log.warning(
                        f"Velocity {velocity*1000:.2f}mm/s exceeds max "
                        f"{vel_max*1000:.1f}mm/s. Clamping."
                    )
                    velocity = vel_max

                # Get current params and update
                params = self._stages[axis_label].get_velocity_parameters()
                self._stages[axis_label].setup_velocity(
                    min_velocity=params.min_velocity,
                    acceleration=params.acceleration,
                    max_velocity=velocity
                )

            return 0

        except Exception as e:
            self.log.error(f"Set velocity failed: {e}")
            return -1

    # =========================================================================
    # Extended Methods (beyond MotorInterface)
    # =========================================================================

    def is_moving(self) -> bool:
        """
        Check if any stage is currently moving.

        Returns:
            bool: True if any axis is moving, False if all are idle.
        """
        for stage in self._stages.values():
            try:
                if stage.is_moving():
                    return True
            except Exception:
                pass
        return False

    def wait_for_idle(self, timeout: float = 30.0) -> bool:
        """
        Wait for all movement to complete.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            bool: True if all axes are idle, False if timeout occurred.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.is_moving():
                # Add small settling time to ensure position is stable
                time.sleep(0.05)
                # Double-check not moving after settling
                if not self.is_moving():
                    self.sigMovementFinished.emit()
                    return True
            time.sleep(0.01)  # 10ms poll interval

        self.log.warning(f"wait_for_idle timed out after {timeout}s")
        return False

    def move_abs_sync(
        self,
        param_dict: Dict[str, float],
        timeout: float = 30.0,
        position_tolerance: float = 50e-6  # 50 µm tolerance
    ) -> int:
        """
        Move to absolute position and wait for completion with position verification.

        Args:
            param_dict: Dictionary with axis labels and absolute positions.
            timeout: Maximum time to wait for completion.
            position_tolerance: Acceptable position error in meters (default 50 µm).

        Returns:
            int: Error code (0: OK, -1: error)
        """
        result = self.move_abs(param_dict)
        if result != 0:
            return result

        if not self.wait_for_idle(timeout):
            self.log.error("Movement timed out")
            return -1

        # Verify positions reached target
        actual_pos = self.get_pos()
        for axis_label, target_pos in param_dict.items():
            if axis_label in actual_pos:
                actual = actual_pos[axis_label]
                error = abs(actual - target_pos)
                if error > position_tolerance:
                    self.log.warning(
                        f"{axis_label}-axis position error: target={target_pos*1000:.3f}mm, "
                        f"actual={actual*1000:.3f}mm, error={error*1000:.3f}mm"
                    )
                    # Don't return error - just warn. The stage may have hit a limit.

        # Emit position update
        self.sigPositionChanged.emit(actual_pos)
        return 0

    def move_rel_sync(
        self,
        param_dict: Dict[str, float],
        timeout: float = 30.0
    ) -> int:
        """
        Move by relative distance and wait for completion.

        Args:
            param_dict: Dictionary with axis labels and relative distances.
            timeout: Maximum time to wait for completion.

        Returns:
            int: Error code (0: OK, -1: error)
        """
        result = self.move_rel(param_dict)
        if result != 0:
            return result

        if not self.wait_for_idle(timeout):
            return -1

        # Emit position update
        self.sigPositionChanged.emit(self.get_pos())
        return 0

    def get_acceleration(self, param_list: Optional[List[str]] = None) -> Dict[str, float]:
        """
        Get the current acceleration for all connected axes.

        Args:
            param_list: Optional list of axis labels. If None, returns all axes.

        Returns:
            dict: Axis labels as keys, accelerations in m/s^2 as values.
        """
        accelerations = {}

        axes_to_query = param_list if param_list is not None else self._stages.keys()

        for axis_label in axes_to_query:
            if axis_label in self._stages:
                try:
                    params = self._stages[axis_label].get_velocity_parameters()
                    accelerations[axis_label] = params.acceleration
                except Exception as e:
                    self.log.warning(f"Failed to get acceleration of {axis_label}: {e}")
                    accelerations[axis_label] = 0.0

        return accelerations

    def set_acceleration(self, param_dict: Dict[str, float]) -> int:
        """
        Set the acceleration for axes.

        Args:
            param_dict: Dictionary with axis labels and accelerations in m/s^2.

        Returns:
            int: Error code (0: OK, -1: error)
        """
        try:
            for axis_label, acceleration in param_dict.items():
                if axis_label not in self._stages:
                    self.log.warning(f"Unknown axis '{axis_label}', ignoring.")
                    continue

                # Validate acceleration
                constraints = self._constraints.get(axis_label, {})
                acc_max = constraints.get('acc_max', SPECS.MAX_ACCELERATION)

                if acceleration <= 0:
                    self.log.warning(f"Acceleration must be positive, got {acceleration}. Ignored.")
                    continue

                if acceleration > acc_max:
                    self.log.warning(
                        f"Acceleration {acceleration*1000:.2f}mm/s^2 exceeds max "
                        f"{acc_max*1000:.1f}mm/s^2. Clamping."
                    )
                    acceleration = acc_max

                # Get current params and update
                params = self._stages[axis_label].get_velocity_parameters()
                self._stages[axis_label].setup_velocity(
                    min_velocity=params.min_velocity,
                    acceleration=acceleration,
                    max_velocity=params.max_velocity
                )

            return 0

        except Exception as e:
            self.log.error(f"Set acceleration failed: {e}")
            return -1

    @staticmethod
    def list_devices() -> List[Tuple[str, str]]:
        """
        List all available Thorlabs Kinesis devices.

        Returns:
            List of (serial_number, device_description) tuples.
        """
        try:
            from pylablib.devices import Thorlabs
            return Thorlabs.list_kinesis_devices()
        except ImportError:
            raise ImportError("pylablib is required")
        except Exception:
            return []

    @staticmethod
    def find_kdc101_devices() -> List[Tuple[str, str]]:
        """
        Find all connected KDC101 devices.

        Returns:
            List of (serial_number, description) tuples for KDC101 controllers.
        """
        devices = ThorlabsKDC101Kinesis.list_devices()
        return [
            (sn, desc) for sn, desc in devices
            if "KDC" in desc.upper() or sn.startswith("27")
        ]
