# -*- coding: utf-8 -*-
"""
Dummy XY motor stage for testing motor scan functionality.

Simulates Thorlabs KDC101-like behavior with asynchronous movement,
position feedback, and wait_for_idle() support.

Also includes PositionAwareInStreamDummy for position-dependent data streaming.

Copyright (c) 2024, the qudi developers.

Example config:

    xy_stage_dummy:
        module.Class: 'dummy.motor_scan_dummy.XYStageDummy'
        options:
            velocity: 0.002          # 2 mm/s movement speed
            movement_delay: 0.1      # seconds between position updates
            position_noise: 0.00001  # 10 µm position noise
    
    position_aware_instream:
        module.Class: 'dummy.motor_scan_dummy.PositionAwareInStreamDummy'
        connect:
            motor_stage: xy_stage_dummy
        options:
            channel_names: ['Signal']
            channel_units: ['V']
            sample_rate: 100.0
            pattern_type: 'sine_diagonal'  # or 'gaussian_spots', 'gradient'
"""

import time
import numpy as np
from typing import Dict, List, Optional, Tuple, Sequence

from threading import Thread, Lock

from PySide2 import QtCore

from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.interface.motor_interface import MotorInterface
from qudi.interface.data_instream_interface import DataInStreamInterface, DataInStreamConstraints
from qudi.interface.data_instream_interface import StreamingMode, SampleTiming
from qudi.util.constraints import ScalarConstraint


class XYStageDummy(MotorInterface):
    """
    Dummy XY motor stage for testing motor scan functionality.
    
    Simulates asynchronous motor movement with position feedback,
    matching the Thorlabs KDC101 API used by MotorScanLogic.
    """
    
    # Config options
    _velocity = ConfigOption(name='velocity', default=0.002)  # 2 mm/s
    _movement_delay = ConfigOption(name='movement_delay', default=0.1)
    _position_noise = ConfigOption(name='position_noise', default=0.00001)  # 10 µm
    
    # Signals
    sigMovementFinished = QtCore.Signal()
    sigPositionChanged = QtCore.Signal(dict)
    sigHomingComplete = QtCore.Signal()
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self._lock = Lock()
        
        # Current positions
        self._positions = {'x': 0.0, 'y': 0.0}
        
        # Target positions
        self._targets = {'x': 0.0, 'y': 0.0}
        
        # Movement state
        self._is_moving = False
        self._movement_thread = None
        self._stop_movement = False
        
        # Constraints
        self._constraints = None
    
    def on_activate(self):
        """Initialize the dummy stage."""
        self._constraints = {
            'x': {
                'label': 'x',
                'unit': 'm',
                'ramp': ['Trapez'],
                'pos_min': 0.0,
                'pos_max': 0.050,  # 50 mm
                'pos_step': 0.000001,  # 1 µm
                'vel_min': 0.0,
                'vel_max': 0.003,  # 3 mm/s
                'vel_step': 0.000001,
                'acc_min': 0.0,
                'acc_max': 0.010,
                'acc_step': 0.000001,
            },
            'y': {
                'label': 'y',
                'unit': 'm',
                'ramp': ['Trapez'],
                'pos_min': 0.0,
                'pos_max': 0.050,  # 50 mm
                'pos_step': 0.000001,
                'vel_min': 0.0,
                'vel_max': 0.003,
                'vel_step': 0.000001,
                'acc_min': 0.0,
                'acc_max': 0.010,
                'acc_step': 0.000001,
            }
        }
        
        self._positions = {'x': 0.0, 'y': 0.0}
        self._targets = {'x': 0.0, 'y': 0.0}
        self._is_moving = False
        
        self.log.info("XYStageDummy activated.")
    
    def on_deactivate(self):
        """Clean up on deactivation."""
        self.abort()
        self.log.info("XYStageDummy deactivated.")
    
    # =========================================================================
    # MotorInterface implementation
    # =========================================================================
    
    def get_constraints(self) -> Dict:
        """Return motor constraints."""
        return self._constraints.copy()
    
    def move_rel(self, param_dict: Dict[str, float]) -> int:
        """Move relative to current position."""
        with self._lock:
            curr_pos = self._positions.copy()
        
        new_targets = {}
        for axis, distance in param_dict.items():
            if axis in self._positions:
                new_targets[axis] = curr_pos[axis] + distance
        
        return self.move_abs(new_targets)
    
    def move_abs(self, param_dict: Dict[str, float]) -> int:
        """
        Start asynchronous movement to absolute position.
        
        Returns immediately, movement continues in background.
        """
        with self._lock:
            # Update targets
            for axis, position in param_dict.items():
                if axis not in self._constraints:
                    continue
                
                constr = self._constraints[axis]
                if position < constr['pos_min'] or position > constr['pos_max']:
                    self.log.warning(f"Position {position*1000:.3f}mm on {axis} "
                                    f"exceeds limits. Clamping.")
                    position = max(constr['pos_min'], min(constr['pos_max'], position))
                
                self._targets[axis] = position
            
            # Start movement thread if not already running
            if not self._is_moving:
                self._stop_movement = False
                self._is_moving = True
                self._movement_thread = Thread(target=self._simulate_movement, daemon=True)
                self._movement_thread.start()
        
        return 0
    
    def abort(self) -> int:
        """Stop all movement immediately."""
        with self._lock:
            self._stop_movement = True
            if self._movement_thread is not None:
                self._movement_thread.join(timeout=1.0)
            self._is_moving = False
            # Set targets to current position
            self._targets = self._positions.copy()
        
        self.log.info("Movement aborted.")
        return 0
    
    def get_pos(self, param_list: Optional[List[str]] = None) -> Dict[str, float]:
        """Get current positions with simulated encoder noise."""
        with self._lock:
            positions = self._positions.copy()
        
        # Add small noise to simulate encoder reading
        result = {}
        axes = param_list if param_list else list(positions.keys())
        
        for axis in axes:
            if axis in positions:
                noise = np.random.normal(0, self._position_noise)
                result[axis] = positions[axis] + noise
        
        return result
    
    def get_status(self, param_list: Optional[List[str]] = None) -> Dict[str, int]:
        """Get axis status. 0 = idle, 1 = moving."""
        with self._lock:
            is_moving = self._is_moving
        
        result = {}
        axes = param_list if param_list else list(self._positions.keys())
        
        for axis in axes:
            result[axis] = 1 if is_moving else 0
        
        return result
    
    def calibrate(self, param_list: Optional[List[str]] = None) -> int:
        """Home the stage (move to 0,0)."""
        axes = param_list if param_list else list(self._positions.keys())
        
        targets = {axis: 0.0 for axis in axes}
        self.move_abs(targets)
        self.wait_for_idle(timeout=30.0)
        
        self.sigHomingComplete.emit()
        return 0
    
    def get_velocity(self, param_list: Optional[List[str]] = None) -> Dict[str, float]:
        """Get configured velocity."""
        result = {}
        axes = param_list if param_list else list(self._positions.keys())
        
        for axis in axes:
            result[axis] = self._velocity
        
        return result
    
    def set_velocity(self, param_dict: Dict[str, float]) -> int:
        """Set velocity (stored but not used in simulation)."""
        # In a real implementation this would affect movement speed
        return 0
    
    # =========================================================================
    # Extended methods (matching Thorlabs KDC101 API)
    # =========================================================================
    
    def is_moving(self) -> bool:
        """Check if any axis is currently moving."""
        with self._lock:
            return self._is_moving
    
    def wait_for_idle(self, timeout: float = 30.0) -> bool:
        """
        Wait for all movement to complete.
        
        Returns:
            bool: True if movement completed, False if timeout.
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.is_moving():
                return True
            time.sleep(0.01)
        
        self.log.warning(f"wait_for_idle timed out after {timeout}s")
        return False
    
    def move_abs_sync(self, param_dict: Dict[str, float], timeout: float = 30.0) -> int:
        """
        Move to position and wait for completion.
        
        Synchronous/blocking version of move_abs.
        """
        result = self.move_abs(param_dict)
        if result != 0:
            return result
        
        if not self.wait_for_idle(timeout):
            return -1
        
        return 0
    
    def move_rel_sync(self, param_dict: Dict[str, float], timeout: float = 30.0) -> int:
        """Move relative and wait for completion."""
        result = self.move_rel(param_dict)
        if result != 0:
            return result
        
        if not self.wait_for_idle(timeout):
            return -1
        
        return 0
    
    # =========================================================================
    # Movement simulation
    # =========================================================================
    
    def _simulate_movement(self):
        """
        Simulate gradual movement toward target position.
        
        Runs in a background thread.
        """
        while not self._stop_movement:
            with self._lock:
                # Check if we've reached all targets
                all_reached = True
                
                for axis in self._positions:
                    current = self._positions[axis]
                    target = self._targets[axis]
                    distance = target - current
                    
                    if abs(distance) > 1e-6:  # 1 µm tolerance
                        all_reached = False
                        
                        # Calculate step based on velocity and delay
                        max_step = self._velocity * self._movement_delay
                        step = min(abs(distance), max_step)
                        
                        if distance < 0:
                            step = -step
                        
                        self._positions[axis] = current + step
                
                if all_reached:
                    self._is_moving = False
                    break
            
            # Emit position update
            self.sigPositionChanged.emit(self.get_pos())
            
            time.sleep(self._movement_delay)
        
        # Final position update
        with self._lock:
            self._is_moving = False
        
        self.sigMovementFinished.emit()
        self.sigPositionChanged.emit(self.get_pos())


class PositionAwareInStreamDummy(DataInStreamInterface):
    """
    Position-aware data stream dummy for motor scan testing.
    
    Generates synthetic data based on motor position, creating a consistent
    2D "fluorescence map" pattern that is the same regardless of scan pattern.
    
    This dummy connects to a motor stage and generates data values based
    on the current position, simulating position-dependent measurements
    like fluorescence or magnetic field scanning.
    
    Example config:
    
        position_aware_instream:
            module.Class: 'dummy.motor_scan_dummy.PositionAwareInStreamDummy'
            connect:
                motor_stage: xy_stage_dummy
            options:
                channel_names:
                    - 'Signal'
                channel_units:
                    - 'V'
                sample_rate: 100.0
                # Pattern: 'sine_diagonal', 'gaussian_spots', or 'gradient'
                pattern_type: 'sine_diagonal'
                # Parameters for sine_diagonal pattern
                sine_wavelength: 0.01  # 10 mm wavelength
                sine_amplitude: 1.0
                sine_offset: 0.5
                noise_level: 0.05
    """
    
    # Connector to motor stage
    _motor_stage = Connector(interface='MotorInterface', name='motor_stage')
    
    # Config options
    _channel_names = ConfigOption(
        name='channel_names',
        default=['Signal'],
        constructor=lambda names: [str(x) for x in names]
    )
    _channel_units = ConfigOption(
        name='channel_units',
        default=['V'],
        constructor=lambda units: [str(x) for x in units]
    )
    _sample_rate = ConfigOption(name='sample_rate', default=100.0)
    _buffer_size = ConfigOption(name='buffer_size', default=1024*1024)
    
    # Pattern configuration
    _pattern_type = ConfigOption(name='pattern_type', default='sine_diagonal')
    _sine_wavelength = ConfigOption(name='sine_wavelength', default=0.01)  # 10 mm
    _sine_amplitude = ConfigOption(name='sine_amplitude', default=1.0)
    _sine_offset = ConfigOption(name='sine_offset', default=0.5)
    _noise_level = ConfigOption(name='noise_level', default=0.05)
    
    # Gaussian spots configuration
    _spot_count = ConfigOption(name='spot_count', default=5)
    _spot_positions = ConfigOption(name='spot_positions', default=None)
    _spot_sigma = ConfigOption(name='spot_sigma', default=0.002)  # 2 mm
    _spot_amplitude = ConfigOption(name='spot_amplitude', default=1.0)
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = Lock()
        self._constraints = None
        self._active_channels = []
        self._is_running = False
        self._sample_buffer = {}
        self._timestamp_buffer = []
        self._buffer_start_time = 0.0
        self._spots = None  # For gaussian_spots pattern
    
    def on_activate(self):
        """Initialize the dummy."""
        # Validate config
        if len(self._channel_names) != len(self._channel_units):
            raise ValueError('channel_names and channel_units must have same length')
        
        # Create constraints
        self._constraints = DataInStreamConstraints(
            channel_units=dict(zip(self._channel_names, self._channel_units)),
            sample_timing=SampleTiming.CONSTANT,
            streaming_modes=[StreamingMode.CONTINUOUS],
            data_type=np.float64,
            channel_buffer_size=ScalarConstraint(
                default=self._buffer_size,
                bounds=(128, 1024**3),
                increment=1,
                enforce_int=True
            ),
            sample_rate=ScalarConstraint(
                default=self._sample_rate,
                bounds=(0.1, 10000),
                increment=0.1
            )
        )
        
        self._active_channels = list(self._channel_names)
        
        # Initialize gaussian spots if needed
        if self._pattern_type == 'gaussian_spots':
            self._initialize_spots()
        
        self.log.info(f"PositionAwareInStreamDummy activated with pattern: {self._pattern_type}")
    
    def on_deactivate(self):
        """Clean up."""
        self._is_running = False
        self._sample_buffer = {}
        self._timestamp_buffer = []
    
    def _initialize_spots(self):
        """Initialize random gaussian spot positions."""
        if self._spot_positions is not None:
            self._spots = np.array(self._spot_positions)
        else:
            # Generate random spot positions within typical scan range
            motor = self._motor_stage()
            constraints = motor.get_constraints()
            
            x_range = (constraints['x']['pos_min'], constraints['x']['pos_max'])
            y_range = (constraints['y']['pos_min'], constraints['y']['pos_max'])
            
            self._spots = np.random.uniform(
                low=[x_range[0], y_range[0]],
                high=[x_range[1], y_range[1]],
                size=(self._spot_count, 2)
            )
    
    def _generate_value_at_position(self, x: float, y: float) -> float:
        """
        Generate a synthetic value based on position.
        
        This creates a consistent 2D pattern that is independent of scan order.
        """
        if self._pattern_type == 'sine_diagonal':
            # Sine wave along diagonal (x + y direction)
            # This creates stripes from top-left to bottom-right
            phase = 2 * np.pi * (x + y) / self._sine_wavelength
            value = self._sine_offset + self._sine_amplitude * np.sin(phase)
            
        elif self._pattern_type == 'gaussian_spots':
            # Sum of gaussian spots
            value = 0.0
            if self._spots is not None:
                for spot_pos in self._spots:
                    dx = x - spot_pos[0]
                    dy = y - spot_pos[1]
                    dist_sq = dx**2 + dy**2
                    value += self._spot_amplitude * np.exp(-dist_sq / (2 * self._spot_sigma**2))
            
        elif self._pattern_type == 'gradient':
            # Simple linear gradient along x
            motor = self._motor_stage()
            constraints = motor.get_constraints()
            x_range = constraints['x']['pos_max'] - constraints['x']['pos_min']
            value = self._sine_offset + self._sine_amplitude * (x / x_range)
            
        else:
            # Default: constant with noise
            value = self._sine_offset
        
        # Add noise
        value += np.random.normal(0, self._noise_level)
        
        return value
    
    # =========================================================================
    # DataInStreamInterface implementation
    # =========================================================================
    
    @property
    def constraints(self) -> DataInStreamConstraints:
        return self._constraints
    
    @property
    def available_samples(self) -> int:
        if not self._is_running:
            return 0
        
        # Generate samples based on elapsed time since start
        elapsed = time.perf_counter() - self._buffer_start_time
        expected_samples = int(elapsed * self._sample_rate)
        
        with self._thread_lock:
            current_samples = len(self._timestamp_buffer)
            return max(0, expected_samples - current_samples)
    
    @property
    def sample_rate(self) -> float:
        return self._sample_rate
    
    @property
    def channel_buffer_size(self) -> int:
        return self._buffer_size
    
    @property
    def streaming_mode(self):
        return StreamingMode.CONTINUOUS
    
    @property 
    def active_channels(self) -> List[str]:
        return self._active_channels.copy()
    
    def configure(self,
                  active_channels: Sequence[str],
                  streaming_mode: StreamingMode,
                  channel_buffer_size: int,
                  sample_rate: float) -> None:
        """Configure the stream."""
        with self._thread_lock:
            self._active_channels = list(active_channels)
            self._sample_rate = sample_rate
            self._buffer_size = channel_buffer_size
    
    def start_stream(self) -> None:
        """Start data streaming."""
        with self._thread_lock:
            self._sample_buffer = {ch: [] for ch in self._active_channels}
            self._timestamp_buffer = []
            self._buffer_start_time = time.perf_counter()
            self._is_running = True
        self.log.debug("Stream started")
    
    def stop_stream(self) -> None:
        """Stop data streaming."""
        with self._thread_lock:
            self._is_running = False
        self.log.debug("Stream stopped")
    
    def read_data_into_buffer(self,
                               data_buffer: np.ndarray,
                               samples_per_channel: int,
                               timestamp_buffer: Optional[np.ndarray] = None) -> int:
        """Read data into provided buffer."""
        if not self._is_running:
            return 0
        
        motor = self._motor_stage()
        pos = motor.get_pos()
        x = pos.get('x', 0.0)
        y = pos.get('y', 0.0)
        
        samples_read = min(samples_per_channel, self.available_samples)
        if samples_read == 0:
            return 0
        
        channel_count = len(self._active_channels)
        current_time = time.perf_counter() - self._buffer_start_time
        
        for i in range(samples_read):
            sample_time = current_time - (samples_read - i - 1) / self._sample_rate
            
            for ch_idx, channel in enumerate(self._active_channels):
                value = self._generate_value_at_position(x, y)
                idx = i * channel_count + ch_idx
                if idx < len(data_buffer):
                    data_buffer[idx] = value
            
            if timestamp_buffer is not None and i < len(timestamp_buffer):
                timestamp_buffer[i] = sample_time
        
        return samples_read
    
    def read_available_data_into_buffer(self,
                                         data_buffer: np.ndarray,
                                         timestamp_buffer: Optional[np.ndarray] = None
                                         ) -> int:
        """Read all available data into buffer."""
        available = self.available_samples
        buffer_capacity = len(data_buffer) // len(self._active_channels)
        samples_to_read = min(available, buffer_capacity)
        return self.read_data_into_buffer(data_buffer, samples_to_read, timestamp_buffer)
    
    def read_data(self,
                  samples_per_channel: Optional[int] = None
                  ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """Read data and return as new arrays."""
        if not self._is_running:
            return np.array([]), None
        
        motor = self._motor_stage()
        pos = motor.get_pos()
        x = pos.get('x', 0.0)
        y = pos.get('y', 0.0)
        
        if samples_per_channel is None:
            samples_per_channel = max(1, self.available_samples)
        
        samples_per_channel = max(1, min(samples_per_channel, 100))
        
        channel_count = len(self._active_channels)
        data = np.empty(samples_per_channel * channel_count, dtype=np.float64)
        timestamps = np.empty(samples_per_channel, dtype=np.float64)
        
        current_time = time.perf_counter() - self._buffer_start_time
        
        for i in range(samples_per_channel):
            sample_time = current_time - (samples_per_channel - i - 1) / self._sample_rate
            timestamps[i] = sample_time
            
            for ch_idx, channel in enumerate(self._active_channels):
                value = self._generate_value_at_position(x, y)
                data[i * channel_count + ch_idx] = value
        
        return data, timestamps
    
    def read_single_point(self) -> Tuple[np.ndarray, Optional[float]]:
        """Read a single data point."""
        motor = self._motor_stage()
        pos = motor.get_pos()
        x = pos.get('x', 0.0)
        y = pos.get('y', 0.0)
        
        data = np.array([self._generate_value_at_position(x, y) 
                        for _ in self._active_channels])
        timestamp = time.perf_counter() - self._buffer_start_time if self._is_running else None
        
        return data, timestamp
