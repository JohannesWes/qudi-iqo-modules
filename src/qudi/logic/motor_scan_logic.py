# -*- coding: utf-8 -*-
"""
Motor-based XY scanning logic module for qudi.

This module provides scanning functionality using motor-controlled stages (e.g., Thorlabs KDC101)
with two data acquisition modes:
1. CONTINUOUS_STREAM: Motors move continuously while streaming data is binned to grid positions
2. STEP_ODMR: Motors stop at each grid position, execute ODMR scans, then proceed

Copyright (c) 2024, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.

Example config:

    motor_scan_logic:
        module.Class: 'motor_scan_logic.MotorScanLogic'
        connect:
            motor_hardware: thorlabs_xy_stage
            odmr_logic: odmr_logic  # optional, for STEP_ODMR mode
            time_series_logic: time_series_reader_logic  # optional, for CONTINUOUS_STREAM mode
        options:
            default_scan_mode: 'STEP_ODMR'  # or 'CONTINUOUS_STREAM'
            position_poll_interval: 0.05  # seconds, for position feedback during continuous scan
            odmr_fit_function: 'fit_hyperfine'  # fitting function to use for ODMR analysis
"""

import time
import os
import sys
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any, Union
from copy import deepcopy
import datetime
import matplotlib.pyplot as plt

from PySide2 import QtCore

from qudi.core.module import LogicBase
from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar
from qudi.util.mutex import RecursiveMutex
from qudi.util.datastorage import TextDataStorage
from qudi.util.units import ScaledFloat

# Add qudi-core root to path to find my_software (same as sensitivity_sweep_logic)
# Get path to qudi-core root (3 levels up from this file)
qudi_core_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if qudi_core_root not in sys.path:
    sys.path.insert(0, qudi_core_root)

# FTW (Frequency Tuning Word) conversion constant for Red Pitaya @ 125 MHz
# FTW = frequency_hz * FTW_PER_HZ; frequency_hz = ftw / FTW_PER_HZ
FTW_PER_HZ = (2**32) / 125e6  # ≈ 34.359738


class ScanMode(Enum):
    """Enumeration of available scan modes."""
    CONTINUOUS_STREAM = 0     # Motors move continuously, data binned by position
    STEP_ODMR = 1             # Motors stop at each point, ODMR scan taken
    CONTINUOUS_FREQ_TRACK = 2 # Motors move continuously, absolute frequency from lock


class ScanPattern(Enum):
    """
    Enumeration of available scan patterns for 2D scans.
    
    The pattern determines the order in which grid points are visited:
    - LINE_BY_LINE: Scan lines along the fast axis, return to start for each new line
    - SNAKE: Scan lines along the fast axis, alternate direction (boustrophedon)
    
    The axis suffix indicates which axis is the "fast" axis (scanned continuously):
    - *_X: Lines along X-axis, step along Y-axis between lines
    - *_Y: Lines along Y-axis, step along X-axis between lines
    """
    LINE_BY_LINE_X = 0  # Scan X lines, return to x_start for each new Y step
    SNAKE_X = 1         # Scan X lines, alternate direction (snake pattern)
    LINE_BY_LINE_Y = 2  # Scan Y lines, return to y_start for each new X step  
    SNAKE_Y = 3         # Scan Y lines, alternate direction (snake pattern)


class ScanState(Enum):
    """Enumeration of scan states."""
    IDLE = 0
    INITIALIZING = 1  # Homing and moving to start position
    RUNNING = 2
    PAUSED = 3
    STOPPING = 4


@dataclass
class MotorScanData:
    """
    Container for motor scan results.
    
    Stores scan configuration and acquired data for both continuous streaming
    and step-and-measure ODMR scan modes.
    """
    # Scan configuration
    scan_axes: Tuple[str, ...]  # e.g., ('x', 'y') or ('x',) for 1D
    scan_range: Tuple[Tuple[float, float], ...]  # inclusive range per axis [(start, stop), ...]
    scan_resolution: Tuple[int, ...]  # points per axis
    scan_mode: ScanMode
    scan_pattern: ScanPattern = ScanPattern.SNAKE_X  # Default to snake pattern along X
    
    # Scan timing
    timestamp_start: Optional[datetime.datetime] = None
    timestamp_end: Optional[datetime.datetime] = None
    scan_duration: float = 0.0  # seconds
    
    # Target positions (grid)
    target_positions: Optional[np.ndarray] = None  # Shape: (n_points, n_axes)
    
    # Actual measured positions (from encoder feedback)
    actual_positions: Optional[np.ndarray] = None  # Shape: (n_points, n_axes) or (n_samples, n_axes)
    
    # For CONTINUOUS_STREAM mode
    # Mean value per grid point for display
    stream_data_mean: Optional[Dict[str, np.ndarray]] = None  # channel -> 2D array (ny, nx)
    # Raw time traces per grid point (variable length)
    stream_data_raw: Optional[Dict[str, List[List[float]]]] = None  # channel -> list of lists
    # Timestamps for streaming data
    stream_timestamps: Optional[List[List[float]]] = None  # list of timestamp lists per point
    
    # For STEP_ODMR mode
    # Averaged ODMR spectrum per grid point
    odmr_frequency_data: Optional[np.ndarray] = None  # 1D frequency array
    odmr_signal_data: Optional[Dict[str, np.ndarray]] = None  # channel -> 3D (ny, nx, n_freq)
    # Fit results per grid point
    odmr_fit_results: Optional[List[List[Dict]]] = None  # 2D list of fit result dicts
    # Raw ODMR data per pixel (for saving individual scans)
    odmr_raw_per_pixel: Optional[List[Dict]] = None  # List of dicts with 'frequency', 'signal' per point
    
    # Derived quantities for display (computed from fits)
    center_frequency: Optional[np.ndarray] = None  # 2D array (ny, nx)
    linewidth: Optional[np.ndarray] = None  # 2D array (ny, nx)
    splitting: Optional[np.ndarray] = None  # 2D array (ny, nx)
    fit_quality: Optional[np.ndarray] = None  # 2D array (ny, nx) - n_features_found
    
    # Scan progress
    current_point_index: int = 0
    total_points: int = 0
    completed: bool = False
    
    def __post_init__(self):
        """Initialize derived attributes after dataclass creation."""
        if self.scan_resolution:
            self.total_points = int(np.prod(self.scan_resolution))
    
    @property
    def scan_dimension(self) -> int:
        """Return scan dimensionality (1 or 2)."""
        return len(self.scan_axes)
    
    @property
    def is_2d(self) -> bool:
        """True if this is a 2D scan."""
        return self.scan_dimension == 2
    
    @property
    def progress(self) -> float:
        """Return scan progress as fraction [0, 1]."""
        if self.total_points == 0:
            return 0.0
        return self.current_point_index / self.total_points
    
    def get_grid_coordinates(self) -> Tuple[np.ndarray, ...]:
        """
        Generate grid coordinate arrays for each axis.
        
        Returns:
            Tuple of 1D arrays, one per scan axis.
        """
        coords = []
        for i, (axis, (start, stop), resolution) in enumerate(
            zip(self.scan_axes, self.scan_range, self.scan_resolution)
        ):
            coords.append(np.linspace(start, stop, resolution))
        return tuple(coords)
    
    def get_flat_target_positions(self) -> np.ndarray:
        """
        Generate flattened array of target positions based on scan pattern.
        
        The scan pattern determines:
        - Which axis is the "fast" axis (scanned within a line)
        - Whether lines alternate direction (snake) or always start from same side
        
        Returns:
            2D array of shape (total_points, n_axes) with (x, y) coordinates.
        """
        coords = self.get_grid_coordinates()
        if not self.is_2d:
            # 1D scan - pattern doesn't matter
            return coords[0].reshape(-1, 1)
        
        x_coords = coords[0]  # Shape: (nx,)
        y_coords = coords[1]  # Shape: (ny,)
        nx = len(x_coords)
        ny = len(y_coords)
        
        positions = []
        
        if self.scan_pattern in (ScanPattern.LINE_BY_LINE_X, ScanPattern.SNAKE_X):
            # Fast axis is X, step along Y between lines
            for iy, y in enumerate(y_coords):
                if self.scan_pattern == ScanPattern.SNAKE_X and iy % 2 == 1:
                    # Reverse X direction for odd lines (snake pattern)
                    line_x = x_coords[::-1]
                else:
                    line_x = x_coords
                for x in line_x:
                    positions.append([x, y])
                    
        elif self.scan_pattern in (ScanPattern.LINE_BY_LINE_Y, ScanPattern.SNAKE_Y):
            # Fast axis is Y, step along X between lines
            for ix, x in enumerate(x_coords):
                if self.scan_pattern == ScanPattern.SNAKE_Y and ix % 2 == 1:
                    # Reverse Y direction for odd lines (snake pattern)
                    line_y = y_coords[::-1]
                else:
                    line_y = y_coords
                for y in line_y:
                    positions.append([x, y])
        
        return np.array(positions)
    
    def point_index_to_grid_index(self, flat_index: int) -> Tuple[int, ...]:
        """
        Convert flat point index to grid indices, accounting for scan pattern.
        
        Uses matrix indexing convention (ij): array[ix, iy] where first index is X.
        
        Args:
            flat_index: Linear index into flattened position array.
            
        Returns:
            Tuple of grid indices (ix, iy) for 2D or (ix,) for 1D.
            These indices correspond to the data array indexing.
        """
        if not self.is_2d:
            return (flat_index,)
            
        nx = self.scan_resolution[0]
        ny = self.scan_resolution[1]
        
        if self.scan_pattern in (ScanPattern.LINE_BY_LINE_X, ScanPattern.SNAKE_X):
            # Fast axis is X: flat_index = iy * nx + ix_in_line
            iy = flat_index // nx
            ix_in_line = flat_index % nx
            if self.scan_pattern == ScanPattern.SNAKE_X and iy % 2 == 1:
                # Reverse mapping for odd lines
                ix = nx - 1 - ix_in_line
            else:
                ix = ix_in_line
            # Return (ix, iy) for matrix indexing convention
            return (ix, iy)
            
        elif self.scan_pattern in (ScanPattern.LINE_BY_LINE_Y, ScanPattern.SNAKE_Y):
            # Fast axis is Y: flat_index = ix * ny + iy_in_line
            ix = flat_index // ny
            iy_in_line = flat_index % ny
            if self.scan_pattern == ScanPattern.SNAKE_Y and ix % 2 == 1:
                # Reverse mapping for odd lines
                iy = ny - 1 - iy_in_line
            else:
                iy = iy_in_line
            # Return (ix, iy) for matrix indexing convention
            return (ix, iy)
        
        # Fallback (should not reach here)
        return (flat_index % nx, flat_index // nx)
    
    def initialize_data_arrays(self, channel_names: List[str] = None):
        """
        Initialize data arrays based on scan configuration.
        
        Args:
            channel_names: List of data channel names for streaming mode.
        """
        self.target_positions = self.get_flat_target_positions()
        self.total_points = len(self.target_positions)
        
        # Initialize actual positions array - same shape as target_positions
        # Will be populated during scanning with encoder feedback
        n_axes = len(self.scan_axes)
        self.actual_positions = np.full((self.total_points, n_axes), np.nan)
        
        if self.is_2d:
            # Use matrix indexing convention (ij): shape = (nx, ny)
            # First index is X, second index is Y
            shape_2d = (self.scan_resolution[0], self.scan_resolution[1])
        else:
            shape_2d = (self.scan_resolution[0],)
        
        if self.scan_mode == ScanMode.CONTINUOUS_STREAM:
            self.stream_data_mean = {}
            self.stream_data_raw = {}
            if channel_names:
                for ch in channel_names:
                    self.stream_data_mean[ch] = np.full(shape_2d, np.nan)
                    self.stream_data_raw[ch] = [[] for _ in range(self.total_points)]
            self.stream_timestamps = [[] for _ in range(self.total_points)]
            
        elif self.scan_mode == ScanMode.STEP_ODMR:
            self.center_frequency = np.full(shape_2d, np.nan)
            self.linewidth = np.full(shape_2d, np.nan)
            self.splitting = np.full(shape_2d, np.nan)
            self.fit_quality = np.full(shape_2d, np.nan)
            self.odmr_fit_results = [[None for _ in range(shape_2d[-1])] 
                                     for _ in range(shape_2d[0] if self.is_2d else 1)]
            # Initialize storage for raw ODMR data per pixel
            self.odmr_raw_per_pixel = [None for _ in range(self.total_points)]
    
    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dictionary for saving."""
        result = {
            'scan_axes': self.scan_axes,
            'scan_range': self.scan_range,
            'scan_resolution': self.scan_resolution,
            'scan_mode': self.scan_mode.name,
            'scan_pattern': self.scan_pattern.name,
            'timestamp_start': self.timestamp_start.isoformat() if self.timestamp_start else None,
            'timestamp_end': self.timestamp_end.isoformat() if self.timestamp_end else None,
            'scan_duration': self.scan_duration,
            'current_point_index': self.current_point_index,
            'total_points': self.total_points,
            'completed': self.completed,
        }
        
        # Add numpy arrays
        if self.target_positions is not None:
            result['target_positions'] = self.target_positions.tolist()
        if self.actual_positions is not None:
            result['actual_positions'] = self.actual_positions.tolist()
            
        # Mode-specific data
        if self.scan_mode == ScanMode.CONTINUOUS_STREAM:
            if self.stream_data_mean:
                result['stream_data_mean'] = {
                    k: v.tolist() for k, v in self.stream_data_mean.items()
                }
            if self.stream_data_raw:
                result['stream_data_raw'] = self.stream_data_raw
                
        elif self.scan_mode == ScanMode.STEP_ODMR:
            if self.odmr_frequency_data is not None:
                result['odmr_frequency_data'] = self.odmr_frequency_data.tolist()
            if self.odmr_signal_data:
                result['odmr_signal_data'] = {
                    k: v.tolist() for k, v in self.odmr_signal_data.items()
                }
            if self.center_frequency is not None:
                result['center_frequency'] = self.center_frequency.tolist()
            if self.linewidth is not None:
                result['linewidth'] = self.linewidth.tolist()
            if self.splitting is not None:
                result['splitting'] = self.splitting.tolist()
            if self.fit_quality is not None:
                result['fit_quality'] = self.fit_quality.tolist()
            if self.odmr_fit_results:
                result['odmr_fit_results'] = self.odmr_fit_results
                
        return result
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MotorScanData':
        """Deserialize from dictionary."""
        mode = ScanMode[data['scan_mode']]
        pattern = ScanPattern[data.get('scan_pattern', 'SNAKE_X')]  # Default for backward compat
        
        instance = cls(
            scan_axes=tuple(data['scan_axes']),
            scan_range=tuple(tuple(r) for r in data['scan_range']),
            scan_resolution=tuple(data['scan_resolution']),
            scan_mode=mode,
            scan_pattern=pattern,
            scan_duration=data.get('scan_duration', 0.0),
            current_point_index=data.get('current_point_index', 0),
            completed=data.get('completed', False),
        )
        
        if data.get('timestamp_start'):
            instance.timestamp_start = datetime.datetime.fromisoformat(data['timestamp_start'])
        if data.get('timestamp_end'):
            instance.timestamp_end = datetime.datetime.fromisoformat(data['timestamp_end'])
            
        if 'target_positions' in data:
            instance.target_positions = np.array(data['target_positions'])
        if 'actual_positions' in data:
            instance.actual_positions = np.array(data['actual_positions'])
            
        # Mode-specific restoration
        if mode == ScanMode.CONTINUOUS_STREAM:
            if 'stream_data_mean' in data:
                instance.stream_data_mean = {
                    k: np.array(v) for k, v in data['stream_data_mean'].items()
                }
            if 'stream_data_raw' in data:
                instance.stream_data_raw = data['stream_data_raw']
                
        elif mode == ScanMode.STEP_ODMR:
            if 'odmr_frequency_data' in data:
                instance.odmr_frequency_data = np.array(data['odmr_frequency_data'])
            if 'odmr_signal_data' in data:
                instance.odmr_signal_data = {
                    k: np.array(v) for k, v in data['odmr_signal_data'].items()
                }
            if 'center_frequency' in data:
                instance.center_frequency = np.array(data['center_frequency'])
            if 'linewidth' in data:
                instance.linewidth = np.array(data['linewidth'])
            if 'splitting' in data:
                instance.splitting = np.array(data['splitting'])
            if 'fit_quality' in data:
                instance.fit_quality = np.array(data['fit_quality'])
            if 'odmr_fit_results' in data:
                instance.odmr_fit_results = data['odmr_fit_results']
                
        return instance
    
    def copy(self) -> 'MotorScanData':
        """Create a deep copy of this scan data."""
        return deepcopy(self)


class MotorScanLogic(LogicBase):
    """
    Logic module for motor-based XY scanning.
    
    Orchestrates motor movement and data acquisition for two modes:
    - CONTINUOUS_STREAM: Motors move continuously, streaming data binned to grid
    - STEP_ODMR: Stop at each point, execute ODMR scan, fit and store results
    """
    
    # Connectors
    _motor_hardware = Connector(interface='MotorInterface', name='motor_hardware')
    _odmr_logic = Connector(interface='LogicBase', name='odmr_logic', optional=True)
    _time_series_logic = Connector(interface='LogicBase',
                                   name='time_series_logic', optional=True)
    _odmr_frequency_tracking_logic = Connector(
        interface='LogicBase',
        name='odmr_frequency_tracking_logic',
        optional=True
    )
    
    # Config options
    _default_scan_mode = ConfigOption(
        name='default_scan_mode',
        default='STEP_ODMR',
        constructor=lambda x: ScanMode[x] if isinstance(x, str) else ScanMode(x)
    )
    _position_poll_interval = ConfigOption(
        name='position_poll_interval',
        default=0.05  # 50 ms
    )
    _odmr_fit_function = ConfigOption(
        name='odmr_fit_function',
        default='fit_hyperfine'
    )
    _require_fit_function = ConfigOption(
        name='require_fit_function',
        default=True,
        missing='info'
    )
    _save_thumbnails = ConfigOption(
        name='save_thumbnails',
        default=True
    )
    # Fit parameters (matching sensitivity_sweep_logic defaults)
    _fit_feature_prominence = ConfigOption(
        name='fit_feature_prominence',
        default=0.02,
        missing='info'
    )
    _fit_n_most_prominent_peaks = ConfigOption(
        name='fit_n_most_prominent_peaks',
        default=5,
        missing='info'
    )
    _fit_min_feature_height = ConfigOption(
        name='fit_min_feature_height',
        default=0.02,
        missing='info'
    )
    _save_odmr_fit_plots = ConfigOption(
        name='save_odmr_fit_plots',
        default=True,
        missing='info'
    )
    _home_before_scan = ConfigOption(
        name='home_before_scan',
        default=False,
        missing='info'
    )
    _lock_status_poll_interval = ConfigOption(
        name='lock_status_poll_interval',
        default=0.5,  # 500 ms - poll lock status during CONTINUOUS_FREQ_TRACK mode
        missing='info'
    )

    # Status variables (persistent across sessions)
    _scan_ranges = StatusVar(
        name='scan_ranges',
        default={'x': (0.0, 0.01), 'y': (0.0, 0.01)}  # 10mm default
    )
    _scan_resolution = StatusVar(
        name='scan_resolution',
        default={'x': 10, 'y': 10}
    )
    _active_scan_mode = StatusVar(name='active_scan_mode', default=None)
    _active_scan_pattern = StatusVar(name='active_scan_pattern', default='SNAKE_X')
    
    # Signals
    sigScanStateChanged = QtCore.Signal(object)  # ScanState
    sigScanDataUpdated = QtCore.Signal()
    sigScanPointCompleted = QtCore.Signal(int, dict)  # point_index, result_dict
    sigScanCompleted = QtCore.Signal(object)  # MotorScanData
    sigPositionUpdated = QtCore.Signal(dict)  # current position dict
    sigScanSettingsChanged = QtCore.Signal(dict)
    sigSaveStateChanged = QtCore.Signal(bool)  # True when saving, False when done
    sigHomingStateChanged = QtCore.Signal(bool)  # True when homing, False when done
    sigMovementStateChanged = QtCore.Signal(bool)  # True when moving to position, False when done
    sigLockLostDuringScan = QtCore.Signal()  # Emitted when lock is lost during CONTINUOUS_FREQ_TRACK
    sigLockStatusUpdated = QtCore.Signal(bool)  # Emitted with current lock status (for GUI indicator)

    # Internal signals for async operations (run on logic thread via QueuedConnection)
    _sigNextPoint = QtCore.Signal()
    _sigDoHoming = QtCore.Signal(object)  # axes list, invokes homing on logic thread
    _sigDoStartScan = QtCore.Signal(object)  # axes list, invokes scan start on logic thread
    _sigDoMove = QtCore.Signal(dict)  # position dict, invokes move on logic thread

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = RecursiveMutex()

        # Scan state
        self._scan_state = ScanState.IDLE
        self._scan_data: Optional[MotorScanData] = None
        self._stop_requested = False

        # Timing
        self._scan_start_time = 0.0

        # Non-blocking operation state
        self._current_point_idx = 0
        self._current_pos_dict = {}
        self._waiting_for_motor = False
        self._waiting_for_odmr = False
        self._odmr_scan_complete = False

        # Timers for non-blocking polling
        self._motor_poll_timer = None
        self._position_timer = None

        # Fit function reference (loaded dynamically)
        self._fit_function = None
        
        # Current scan folder for saving fit plots
        self._current_scan_folder = None
        
        # Time series streaming state
        self._ts_we_started = False  # True if we started the time series reader
        self._ts_raw_data_buffer = {}  # Buffer for incoming raw data per channel
        self._ts_connected = False  # Track if we're connected to sigNewRawData

        # CONTINUOUS_FREQ_TRACK mode state
        self._tracking_zero_crossing = None  # Hz, baseline from tracking logic at scan start
        self._tracking_zero_crossing_history = []  # List of (point_idx, zero_crossing) for segments
        self._paused_due_to_lock_loss = False  # Distinguish auto-pause from user pause
        self._lock_status_timer = None  # Timer for polling lock status

        # Non-blocking homing state
        self._homing_poll_timer = None
        self._homing_in_progress = False
        self._homing_axes = []
        self._homing_start_time = 0.0
        self._homing_timeout = 120.0  # seconds

        # Non-blocking manual movement state
        self._moving_in_progress = False

    def on_activate(self):
        """Initialize the module."""
        # Validate connectors
        motor = self._motor_hardware()
        if motor is None:
            self.log.error("Motor hardware connector not available!")
            return
        
        # Get motor constraints
        self._motor_constraints = motor.get_constraints()
        self.log.info(f"Motor axes available: {list(self._motor_constraints.keys())}")
        
        # Set default scan mode if not set
        if self._active_scan_mode is None:
            self._active_scan_mode = self._default_scan_mode.value
        
        # Try to load fit function
        self._load_fit_function()
        
        # Connect internal signals
        self._sigNextPoint.connect(self._process_next_point, QtCore.Qt.QueuedConnection)
        self._sigDoHoming.connect(self._do_homing_async, QtCore.Qt.QueuedConnection)
        self._sigDoStartScan.connect(self._do_start_scan_async, QtCore.Qt.QueuedConnection)
        self._sigDoMove.connect(self._do_move_async, QtCore.Qt.QueuedConnection)

        # Initialize motor poll timer (non-blocking motor movement check)
        self._motor_poll_timer = QtCore.QTimer()
        self._motor_poll_timer.setSingleShot(True)
        self._motor_poll_timer.timeout.connect(self._on_motor_poll_timeout, QtCore.Qt.QueuedConnection)

        # Initialize position timer (for continuous mode data collection)
        self._position_timer = QtCore.QTimer()
        self._position_timer.setSingleShot(True)
        self._position_timer.timeout.connect(self._on_position_poll_timeout, QtCore.Qt.QueuedConnection)

        # Initialize lock status polling timer (for CONTINUOUS_FREQ_TRACK mode)
        self._lock_status_timer = QtCore.QTimer()
        self._lock_status_timer.setSingleShot(False)  # Repeating timer
        self._lock_status_timer.timeout.connect(
            self._on_lock_status_poll_timeout, QtCore.Qt.QueuedConnection
        )

        # Initialize homing poll timer (non-blocking homing check)
        self._homing_poll_timer = QtCore.QTimer()
        self._homing_poll_timer.setSingleShot(True)
        self._homing_poll_timer.timeout.connect(
            self._on_homing_poll_timeout, QtCore.Qt.QueuedConnection
        )

        self.log.info("MotorScanLogic activated.")
    
    def on_deactivate(self):
        """Clean up on deactivation."""
        # Stop any running scan
        if self._scan_state != ScanState.IDLE:
            self.stop_scan()

        # Stop timers
        if self._motor_poll_timer is not None:
            self._motor_poll_timer.stop()
        if self._position_timer is not None:
            self._position_timer.stop()
        if self._lock_status_timer is not None:
            self._lock_status_timer.stop()
        if self._homing_poll_timer is not None:
            self._homing_poll_timer.stop()

        # Disconnect ODMR signals if connected
        odmr = self._odmr_logic()
        if odmr is not None and hasattr(odmr, 'sigScanStateUpdated'):
            try:
                odmr.sigScanStateUpdated.disconnect(self._on_odmr_scan_state_changed)
            except (TypeError, RuntimeError):
                pass

        # Disconnect time series signals if connected
        self._disconnect_time_series_signals()

        # Disconnect signals
        try:
            self._sigNextPoint.disconnect()
        except:
            pass
        try:
            self._sigDoHoming.disconnect()
        except:
            pass
        try:
            self._sigDoStartScan.disconnect()
        except:
            pass
        try:
            self._sigDoMove.disconnect()
        except:
            pass

        self.log.info("MotorScanLogic deactivated.")
    
    def _connect_time_series_signals(self):
        """Connect to time series logic signals for non-blocking data reception."""
        if self._ts_connected:
            return
        
        ts_logic = self._time_series_logic()
        if ts_logic is not None and hasattr(ts_logic, 'sigNewRawData'):
            try:
                ts_logic.sigNewRawData.connect(
                    self._on_new_raw_data,
                    QtCore.Qt.QueuedConnection
                )
                self._ts_connected = True
                self.log.debug("Connected to time_series_logic.sigNewRawData")
            except (TypeError, RuntimeError) as e:
                self.log.warning(f"Could not connect to sigNewRawData: {e}")
    
    def _disconnect_time_series_signals(self):
        """Disconnect from time series logic signals."""
        if not self._ts_connected:
            return
        
        ts_logic = self._time_series_logic()
        if ts_logic is not None and hasattr(ts_logic, 'sigNewRawData'):
            try:
                ts_logic.sigNewRawData.disconnect(self._on_new_raw_data)
                self.log.debug("Disconnected from time_series_logic.sigNewRawData")
            except (TypeError, RuntimeError):
                pass
        self._ts_connected = False
    
    @QtCore.Slot(object, object)
    def _on_new_raw_data(self, data_buffer, times_buffer):
        """
        Handle new raw data from time series logic.
        
        This slot receives data continuously from the time_series_logic's sigNewRawData
        signal, allowing non-blocking data collection while the motor is moving.
        The data is buffered and associated with the current scan position.
        
        Args:
            data_buffer: Raw data array from streamer (flattened, all channels)
            times_buffer: Optional timestamp array
        """
        if self._scan_state != ScanState.RUNNING:
            return

        if self._scan_data is None:
            return

        mode = self._scan_data.scan_mode

        # Only process data for continuous streaming modes
        if mode not in (ScanMode.CONTINUOUS_STREAM, ScanMode.CONTINUOUS_FREQ_TRACK):
            return

        # Get channel information from time series logic
        ts_logic = self._time_series_logic()
        if ts_logic is None:
            return

        try:
            if mode == ScanMode.CONTINUOUS_FREQ_TRACK:
                # For frequency tracking, we receive FTW values and convert to absolute frequency
                self._process_freq_track_data(data_buffer)
            else:
                # Standard CONTINUOUS_STREAM processing
                channel_names = ts_logic.active_channel_names
                if not channel_names:
                    return

                # Parse the flattened data buffer into per-channel data
                # data_buffer is flattened: [ch1_s1, ch2_s1, ch1_s2, ch2_s2, ...]
                n_channels = len(channel_names)
                if data_buffer is None or len(data_buffer) == 0:
                    return

                n_samples = len(data_buffer) // n_channels
                if n_samples == 0:
                    return

                # Reshape to (n_samples, n_channels) then transpose to (n_channels, n_samples)
                data_reshaped = data_buffer[:n_samples * n_channels].reshape(n_samples, n_channels).T

                # Buffer the data for each channel
                for i, ch_name in enumerate(channel_names):
                    if ch_name not in self._ts_raw_data_buffer:
                        self._ts_raw_data_buffer[ch_name] = []
                    self._ts_raw_data_buffer[ch_name].extend(data_reshaped[i].tolist())

        except Exception as e:
            self.log.debug(f"Error processing raw data: {e}")

    def _process_freq_track_data(self, data_buffer):
        """
        Process FTW data for CONTINUOUS_FREQ_TRACK mode.

        Converts raw FTW (Frequency Tuning Word) values from the FPGA to
        absolute frequency by adding the zero-crossing baseline.

        Args:
            data_buffer: Raw data array from streamer (FTW values, signed 32-bit)
        """
        import numpy as np

        if data_buffer is None or len(data_buffer) == 0:
            return

        if self._tracking_zero_crossing is None:
            self.log.warning("No zero-crossing baseline set for frequency tracking")
            return

        # Get time series logic for channel info
        ts_logic = self._time_series_logic()
        if ts_logic is None:
            return

        try:
            # Time series may have multiple channels, but we only use the first (FTW correction)
            channel_names = ts_logic.active_channel_names
            n_channels = len(channel_names) if channel_names else 1
            n_samples = len(data_buffer) // n_channels
            if n_samples == 0:
                return

            # Extract first channel data (FTW values)
            if n_channels > 1:
                ftw_data = data_buffer[:n_samples * n_channels].reshape(n_samples, n_channels)[:, 0]
            else:
                ftw_data = data_buffer[:n_samples]

            # Convert FTW to frequency correction in Hz
            # FTW is a signed 32-bit value representing frequency in FPGA units
            ftw_array = np.asarray(ftw_data, dtype=np.float64)
            correction_hz = ftw_array / FTW_PER_HZ

            # Calculate absolute frequency = zero-crossing + correction
            absolute_freq_hz = self._tracking_zero_crossing + correction_hz

            # Buffer for the 'absolute_frequency' channel
            if 'absolute_frequency' not in self._ts_raw_data_buffer:
                self._ts_raw_data_buffer['absolute_frequency'] = []
            self._ts_raw_data_buffer['absolute_frequency'].extend(absolute_freq_hz.tolist())

        except Exception as e:
            self.log.debug(f"Error processing frequency tracking data: {e}")

    def _load_fit_function(self):
        """Load the ODMR fit function from my_software.tools.fitting."""
        try:
            from my_software.tools.fitting import fit_hyperfine
            self._fit_function = fit_hyperfine
            self.log.info("Loaded fit_hyperfine function from my_software.tools.fitting")
        except ImportError as e:
            if self._require_fit_function:
                self.log.warning(f"Could not load fit function: {e}. ODMR fitting will be unavailable.")
            else:
                self.log.debug(f"Fit function not loaded (optional): {e}")
            self._fit_function = None
    
    # =========================================================================
    # Properties
    # =========================================================================
    
    @property
    def scan_state(self) -> ScanState:
        """Current scan state."""
        return self._scan_state
    
    @property
    def is_scanning(self) -> bool:
        """True if a scan is in progress (including initialization phase)."""
        return self._scan_state in (ScanState.INITIALIZING, ScanState.RUNNING, ScanState.PAUSED)
    
    @property
    def scan_data(self) -> Optional[MotorScanData]:
        """Current scan data."""
        with self._thread_lock:
            return self._scan_data
    
    @property
    def motor_constraints(self) -> Dict:
        """Motor hardware constraints."""
        return self._motor_constraints.copy()
    
    @property
    def available_axes(self) -> List[str]:
        """List of available motor axes."""
        return list(self._motor_constraints.keys())
    
    @property
    def scan_settings(self) -> Dict:
        """Current scan settings."""
        return {
            'scan_mode': self._active_scan_mode,
            'scan_pattern': self._active_scan_pattern,
            'scan_ranges': self._scan_ranges.copy(),
            'scan_resolution': self._scan_resolution.copy(),
        }
    
    @property
    def current_position(self) -> Dict[str, float]:
        """Get current motor position."""
        try:
            return self._motor_hardware().get_pos()
        except Exception as e:
            self.log.warning(f"Failed to get motor position: {e}")
            return {}
    
    @property
    def scan_pattern(self) -> ScanPattern:
        """Current scan pattern."""
        if isinstance(self._active_scan_pattern, str):
            return ScanPattern[self._active_scan_pattern]
        return ScanPattern(self._active_scan_pattern)
    
    # =========================================================================
    # Settings Methods
    # =========================================================================
    
    @QtCore.Slot(str)
    def set_scan_mode(self, mode: Union[str, ScanMode]):
        """Set the scan mode."""
        with self._thread_lock:
            if self.is_scanning:
                self.log.error("Cannot change scan mode while scanning.")
                return
            
            if isinstance(mode, str):
                mode = ScanMode[mode]
            
            self._active_scan_mode = mode.value
            self.sigScanSettingsChanged.emit(self.scan_settings)
            self.log.info(f"Scan mode set to: {mode.name}")
    
    @QtCore.Slot(str)
    def set_scan_pattern(self, pattern: Union[str, ScanPattern]):
        """
        Set the scan pattern.
        
        Args:
            pattern: Scan pattern (LINE_BY_LINE_X, SNAKE_X, LINE_BY_LINE_Y, SNAKE_Y)
        """
        with self._thread_lock:
            if self.is_scanning:
                self.log.error("Cannot change scan pattern while scanning.")
                return
            
            if isinstance(pattern, str):
                pattern = ScanPattern[pattern]
            
            self._active_scan_pattern = pattern.name
            self.sigScanSettingsChanged.emit(self.scan_settings)
            self.log.info(f"Scan pattern set to: {pattern.name}")
    
    @QtCore.Slot(dict)
    def set_scan_ranges(self, ranges: Dict[str, Tuple[float, float]]):
        """
        Set scan ranges for each axis.
        
        Args:
            ranges: Dict mapping axis name to (start, stop) tuple.
        """
        with self._thread_lock:
            if self.is_scanning:
                self.log.error("Cannot change scan ranges while scanning.")
                return
            
            # Validate against motor constraints
            for axis, (start, stop) in ranges.items():
                if axis not in self._motor_constraints:
                    self.log.warning(f"Axis '{axis}' not in motor constraints, ignoring.")
                    continue
                
                constraints = self._motor_constraints[axis]
                pos_min = constraints.get('pos_min', -float('inf'))
                pos_max = constraints.get('pos_max', float('inf'))
                
                # Clamp to valid range
                start = max(pos_min, min(pos_max, start))
                stop = max(pos_min, min(pos_max, stop))
                
                self._scan_ranges[axis] = (start, stop)
            
            self.sigScanSettingsChanged.emit(self.scan_settings)
    
    @QtCore.Slot(dict)
    def set_scan_resolution(self, resolution: Dict[str, int]):
        """
        Set scan resolution (number of points) for each axis.
        
        Args:
            resolution: Dict mapping axis name to number of points.
        """
        with self._thread_lock:
            if self.is_scanning:
                self.log.error("Cannot change scan resolution while scanning.")
                return
            
            for axis, points in resolution.items():
                if axis in self._scan_ranges:
                    self._scan_resolution[axis] = max(1, int(points))
            
            self.sigScanSettingsChanged.emit(self.scan_settings)
    
    # =========================================================================
    # Motor Control Methods
    # =========================================================================
    
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
    def _on_lock_status_poll_timeout(self):
        """
        Poll lock status during CONTINUOUS_FREQ_TRACK mode.

        Auto-pauses the scan if lock is lost, emitting sigLockLostDuringScan.
        Emits sigLockStatusUpdated for GUI indicator updates.
        """
        if self._scan_state != ScanState.RUNNING:
            return

        if self._active_scan_mode != ScanMode.CONTINUOUS_FREQ_TRACK.value:
            return

        tracking_logic = self._odmr_frequency_tracking_logic()
        if tracking_logic is None:
            return

        # Get current lock status
        try:
            lock_enabled = tracking_logic.lock_enabled
        except Exception as e:
            self.log.warning(f"Could not read lock status: {e}")
            return

        # Emit status update for GUI
        self.sigLockStatusUpdated.emit(lock_enabled)

        # Check for lock loss
        if not lock_enabled:
            self.log.warning("Lock lost during frequency tracking scan - pausing scan")
            self._paused_due_to_lock_loss = True
            self._scan_state = ScanState.PAUSED
            self._lock_status_timer.stop()
            self.sigScanStateChanged.emit(self._scan_state.name)
            self.sigLockLostDuringScan.emit()

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
    # Scan Control Methods  
    # =========================================================================
    
    @QtCore.Slot(list)
    def start_scan(self, axes: List[str] = None):
        """
        Start a new scan (non-blocking).
        
        This method returns immediately. The actual scan setup and execution
        runs asynchronously on the logic thread. If home_before_scan is enabled,
        homes the stages before starting the scan.
        
        Args:
            axes: List of axes to scan. Defaults to ['x', 'y'] or ['x'] based on config.
        """
        with self._thread_lock:
            if self.is_scanning:
                self.log.error("Scan already in progress.")
                return
            
            if self._homing_in_progress:
                self.log.error("Cannot start scan while homing is in progress.")
                return
            
            # Default to available axes
            if axes is None:
                axes = [a for a in ['x', 'y'] if a in self._scan_ranges]
            
            # Quick validation of axes
            for axis in axes:
                if axis not in self._scan_ranges:
                    self.log.error(f"Axis '{axis}' not configured. Available: {list(self._scan_ranges.keys())}")
                    return
            
            # Mark as initializing (prevents double-start, disables UI)
            self._scan_state = ScanState.INITIALIZING
            self.sigScanStateChanged.emit(self._scan_state)
            
            # Emit signal to run scan setup asynchronously on logic thread
            self._sigDoStartScan.emit(list(axes))

    @QtCore.Slot(object)
    def _do_start_scan_async(self, axes: List[str]):
        """
        Perform the actual scan setup and start (runs on logic thread via signal).
        
        This method is invoked via _sigDoStartScan with QueuedConnection to ensure
        it runs on the logic's thread, not blocking the GUI thread.
        
        Args:
            axes: List of axes to scan.
        """
        try:
            # Home stages before scan if configured
            if self._home_before_scan:
                self.log.info("Homing stages before scan...")
                motor = self._motor_hardware()
                if motor is not None:
                    try:
                        result = motor.calibrate(axes)
                        if result != 0:
                            self.log.warning(f"Homing returned error code {result}. Proceeding anyway.")
                    except Exception as e:
                        self.log.error(f"Homing failed: {e}. Aborting scan.")
                        self._scan_state = ScanState.IDLE
                        self.sigScanStateChanged.emit(self._scan_state)
                        return
                else:
                    self.log.error("Motor hardware not available. Aborting scan.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
            
            # Determine scan mode
            mode = ScanMode(self._active_scan_mode) if isinstance(self._active_scan_mode, int) else self._active_scan_mode
            
            # Check required connectors for the mode
            if mode == ScanMode.STEP_ODMR:
                if self._odmr_logic() is None:
                    self.log.error("ODMR logic not connected. Cannot perform STEP_ODMR scan.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
            elif mode == ScanMode.CONTINUOUS_STREAM:
                if self._time_series_logic() is None:
                    self.log.error("Time series logic not connected. Cannot perform CONTINUOUS_STREAM scan.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
            elif mode == ScanMode.CONTINUOUS_FREQ_TRACK:
                # Validate all required components for frequency tracking
                tracking_logic = self._odmr_frequency_tracking_logic()
                if tracking_logic is None:
                    self.log.error("ODMR frequency tracking logic not connected. "
                                   "Cannot perform CONTINUOUS_FREQ_TRACK scan.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
                if self._time_series_logic() is None:
                    self.log.error("Time series logic not connected. "
                                   "Cannot perform CONTINUOUS_FREQ_TRACK scan.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
                # Check lock is enabled
                if not tracking_logic.lock_enabled:
                    self.log.error("Lock not enabled. Enable frequency tracking in the "
                                   "ODMR Tracking GUI before starting scan.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
                # Check zero-crossing is available
                zero_crossing = tracking_logic.zero_crossing_frequency
                if zero_crossing is None:
                    self.log.error("No zero-crossing frequency available. Perform ODMR fit "
                                   "in Tracking GUI first.")
                    self._scan_state = ScanState.IDLE
                    self.sigScanStateChanged.emit(self._scan_state)
                    return
                # Store baseline for this scan
                self._tracking_zero_crossing = zero_crossing
                self._tracking_zero_crossing_history = [(0, zero_crossing)]
                self._paused_due_to_lock_loss = False
                self.log.info(f"Frequency tracking scan with zero-crossing: "
                              f"{zero_crossing / 1e9:.6f} GHz")
            
            # Create scan configuration
            scan_axes = tuple(axes)
            scan_range = tuple(self._scan_ranges[a] for a in axes)
            scan_resolution = tuple(self._scan_resolution[a] for a in axes)
            
            # Get current scan pattern
            pattern = self.scan_pattern
            
            # Create scan data container
            self._scan_data = MotorScanData(
                scan_axes=scan_axes,
                scan_range=scan_range,
                scan_resolution=scan_resolution,
                scan_mode=mode,
                scan_pattern=pattern,
                timestamp_start=datetime.datetime.now()
            )
            
            # Initialize data arrays and set up data acquisition
            if mode == ScanMode.CONTINUOUS_STREAM:
                ts_logic = self._time_series_logic()
                channel_names = ts_logic.active_channel_names if ts_logic else []
                self._scan_data.initialize_data_arrays(channel_names)

                # Clear raw data buffer
                self._ts_raw_data_buffer = {ch: [] for ch in channel_names}

                # Check if time series is already running (e.g., from time_series GUI)
                ts_already_running = ts_logic.module_state() == 'locked' if ts_logic else False

                if ts_already_running:
                    # Time series already running - just connect to its signals
                    self._ts_we_started = False
                else:
                    # Start the time series reader ourselves
                    self._ts_we_started = True
                    if ts_logic is not None:
                        ts_logic.start_reading()

                # Connect to sigNewRawData for non-blocking data reception
                self._connect_time_series_signals()
                self._current_scan_folder = None  # Will be set on save
            elif mode == ScanMode.CONTINUOUS_FREQ_TRACK:
                # Initialize with absolute frequency channel
                channel_names = ['absolute_frequency']
                self._scan_data.initialize_data_arrays(channel_names)

                # Clear raw data buffer (will receive FTW values, convert to frequency)
                self._ts_raw_data_buffer = {ch: [] for ch in channel_names}

                # Configure tracking logic for FTW streaming
                tracking_logic = self._odmr_frequency_tracking_logic()
                if tracking_logic is not None:
                    try:
                        tracking_logic.set_stream_mode('correction')
                        self.log.debug("Set tracking stream mode to 'correction' (FTW output)")
                    except Exception as e:
                        self.log.warning(f"Could not set stream mode: {e}")

                # Connect to time series for data (same as CONTINUOUS_STREAM)
                ts_logic = self._time_series_logic()
                ts_already_running = ts_logic.module_state() == 'locked' if ts_logic else False

                if ts_already_running:
                    self._ts_we_started = False
                else:
                    self._ts_we_started = True
                    if ts_logic is not None:
                        ts_logic.start_reading()

                self._connect_time_series_signals()
                self._current_scan_folder = None

                # Start lock status polling
                poll_interval_ms = int(self._lock_status_poll_interval * 1000)
                self._lock_status_timer.start(poll_interval_ms)
            else:
                self._scan_data.initialize_data_arrays()
                # Create scan folder now for STEP_ODMR mode so fit plots can be saved during scan
                if self._save_odmr_fit_plots:
                    timestamp = datetime.datetime.now()
                    timestamp_str = timestamp.strftime('%Y%m%d-%H%M-%S')
                    nametag = f'motor_scan_{mode.name}'
                    scan_folder_name = f'{timestamp_str}_{nametag}'
                    self._current_scan_folder = os.path.join(self.module_default_data_dir, scan_folder_name)
                    os.makedirs(self._current_scan_folder, exist_ok=True)
                else:
                    self._current_scan_folder = None
            
            # Set state to RUNNING now that initialization is complete
            self._stop_requested = False
            self._scan_start_time = time.time()
            self._scan_state = ScanState.RUNNING
            
            self.module_state.lock()
            self.sigScanStateChanged.emit(self._scan_state)
            
            self.log.info(f"Starting {mode.name} scan on axes {axes} "
                         f"with pattern {pattern.name}, resolution {scan_resolution}")
            
            # Start the scan loop
            self._sigNextPoint.emit()
            
        except Exception as e:
            self.log.error(f"Failed to start scan: {e}", exc_info=True)
            self._scan_state = ScanState.IDLE
            self.sigScanStateChanged.emit(self._scan_state)
    
    @QtCore.Slot()
    def stop_scan(self):
        """Stop the current scan."""
        with self._thread_lock:
            if not self.is_scanning:
                return
            
            self._stop_requested = True
            self._scan_state = ScanState.STOPPING
            self.sigScanStateChanged.emit(self._scan_state)
            self.log.info("Scan stop requested.")
    
    @QtCore.Slot()
    def pause_scan(self):
        """Pause the current scan."""
        with self._thread_lock:
            if self._scan_state != ScanState.RUNNING:
                return
            
            self._scan_state = ScanState.PAUSED
            self.sigScanStateChanged.emit(self._scan_state)
            self.log.info("Scan paused.")
    
    @QtCore.Slot()
    def resume_scan(self):
        """Resume a paused scan."""
        with self._thread_lock:
            if self._scan_state != ScanState.PAUSED:
                return

            mode = ScanMode(self._active_scan_mode) if isinstance(self._active_scan_mode, int) else self._active_scan_mode

            # For CONTINUOUS_FREQ_TRACK mode, validate lock and update zero-crossing
            if mode == ScanMode.CONTINUOUS_FREQ_TRACK:
                tracking_logic = self._odmr_frequency_tracking_logic()
                if tracking_logic is None:
                    self.log.error("Tracking logic not available. Cannot resume.")
                    return

                # Validate lock is re-enabled
                if not tracking_logic.lock_enabled:
                    self.log.error("Lock not enabled. Re-enable tracking in ODMR Tracking GUI before resuming.")
                    return

                # Get current zero-crossing (may have changed after re-locking)
                new_zero_crossing = tracking_logic.zero_crossing_frequency
                if new_zero_crossing is None:
                    self.log.error("No zero-crossing frequency. Perform ODMR fit before resuming.")
                    return

                # Check if zero-crossing changed and record in history
                if new_zero_crossing != self._tracking_zero_crossing:
                    current_point = self._scan_data.current_point_index if self._scan_data else 0
                    old_zc = self._tracking_zero_crossing
                    self.log.info(f"Zero-crossing updated: {old_zc / 1e9:.6f} GHz → "
                                  f"{new_zero_crossing / 1e9:.6f} GHz")
                    self._tracking_zero_crossing_history.append((current_point, new_zero_crossing))
                    self._tracking_zero_crossing = new_zero_crossing

                # Reset lock loss flag and restart lock status polling
                self._paused_due_to_lock_loss = False
                poll_interval_ms = int(self._lock_status_poll_interval * 1000)
                self._lock_status_timer.start(poll_interval_ms)

            self._scan_state = ScanState.RUNNING
            self.sigScanStateChanged.emit(self._scan_state)
            self.log.info("Scan resumed.")
            self._sigNextPoint.emit()
    
    @QtCore.Slot()
    def _process_next_point(self):
        """Process the next point in the scan (non-blocking, called via signal queue)."""
        with self._thread_lock:
            # Check if we should stop
            if self._stop_requested or self._scan_state == ScanState.STOPPING:
                # Stop the motor when aborting scan
                motor = self._motor_hardware()
                if motor is not None:
                    motor.abort()
                self._finalize_scan(completed=False)
                return

            # Check if paused
            if self._scan_state == ScanState.PAUSED:
                return  # Will resume via resume_scan()

            # Check if scan is complete
            if self._scan_data.current_point_index >= self._scan_data.total_points:
                self._finalize_scan(completed=True)
                return

            # Get current point
            self._current_point_idx = self._scan_data.current_point_index
            target_pos = self._scan_data.target_positions[self._current_point_idx]

            # Build position dict
            self._current_pos_dict = {axis: pos for axis, pos in
                                      zip(self._scan_data.scan_axes, target_pos)}

            # Start motor movement (non-blocking)
            motor = self._motor_hardware()
            motor.move_abs(self._current_pos_dict)

            # Set state and start polling timer
            self._waiting_for_motor = True
            self._motor_poll_timer.start(50)  # Poll every 50ms

    @QtCore.Slot()
    def _on_motor_poll_timeout(self):
        """Called periodically to check if motor has finished moving."""
        with self._thread_lock:
            if self._stop_requested or self._scan_state == ScanState.STOPPING:
                self._waiting_for_motor = False
                # Stop the motor when aborting scan to prevent issues with subsequent homing
                motor = self._motor_hardware()
                if motor is not None:
                    motor.abort()
                self._finalize_scan(completed=False)
                return

            motor = self._motor_hardware()

            # Check if motor is still moving
            is_moving = False
            if hasattr(motor, 'is_moving'):
                is_moving = motor.is_moving()
            else:
                status = motor.get_status()
                is_moving = any(v != 0 for v in status.values())

            if is_moving:
                self._motor_poll_timer.start(50)
                return

            # Verify position is within tolerance of target
            if hasattr(motor, 'get_pos'):
                actual_pos = motor.get_pos()
                position_tolerance = 100e-6  # 100 µm
                
                position_ok = all(
                    abs(actual_pos.get(axis, 0) - target) <= position_tolerance
                    for axis, target in self._current_pos_dict.items()
                )
                
                if not position_ok:
                    # Wait up to 2 seconds for position to settle
                    if not hasattr(self, '_position_settle_start'):
                        self._position_settle_start = time.time()
                    
                    if time.time() - self._position_settle_start < 2.0:
                        self._motor_poll_timer.start(100)
                        return
                    else:
                        # Timeout - log warning
                        for axis, target in self._current_pos_dict.items():
                            error = abs(actual_pos.get(axis, 0) - target)
                            if error > position_tolerance:
                                self.log.warning(
                                    f"Position error on {axis}: target={target*1000:.2f}mm, "
                                    f"actual={actual_pos.get(axis, 0)*1000:.2f}mm"
                                )
                
                if hasattr(self, '_position_settle_start'):
                    delattr(self, '_position_settle_start')
                
                # Log position at measurement point
                point_idx = self._scan_data.current_point_index
                total_points = len(self._scan_data.target_positions)
                pos_str = ", ".join(f"{a}={actual_pos.get(a, 0)*1000:.2f}" for a in self._current_pos_dict)
                self.log.info(f"Point {point_idx+1}/{total_points} at ({pos_str})mm")
            
            self._waiting_for_motor = False

            if self._scan_data.scan_mode == ScanMode.STEP_ODMR:
                self._start_odmr_scan_async()
            else:
                self._collect_continuous_data_and_advance()

    def _start_odmr_scan_async(self):
        """Start ODMR scan asynchronously (non-blocking)."""
        odmr = self._odmr_logic()
        if odmr is None:
            self.log.error("ODMR logic not available")
            self._advance_to_next_point({})
            return

        # Set position hint for ODMR dummy
        if hasattr(odmr, 'set_position_hint'):
            odmr.set_position_hint(self._current_pos_dict)

        # Connect to ODMR completion signal
        self._odmr_scan_complete = False
        self._waiting_for_odmr = True

        try:
            odmr.sigScanStateUpdated.connect(
                self._on_odmr_scan_state_changed,
                QtCore.Qt.QueuedConnection
            )
        except (TypeError, RuntimeError):
            pass  # Already connected

        # Start ODMR scan
        odmr.start_odmr_scan()

    @QtCore.Slot(bool)
    def _on_odmr_scan_state_changed(self, is_running: bool):
        """Called when ODMR scan state changes."""
        if not self._waiting_for_odmr:
            return

        if not is_running:
            # ODMR scan completed
            self._waiting_for_odmr = False
            self._odmr_scan_complete = True

            # Disconnect signal to avoid multiple calls
            odmr = self._odmr_logic()
            if odmr is not None:
                try:
                    odmr.sigScanStateUpdated.disconnect(self._on_odmr_scan_state_changed)
                except (TypeError, RuntimeError):
                    pass

            # Process ODMR results and advance
            result = self._process_odmr_results()
            self._advance_to_next_point(result)

    def _process_odmr_results(self) -> Dict:
        """Process ODMR scan results and extract data using fit_hyperfine."""
        odmr = self._odmr_logic()
        motor = self._motor_hardware()
        point_idx = self._current_point_idx
        position = self._current_pos_dict

        # Get actual position
        actual_pos = motor.get_pos()

        # Get ODMR data
        frequency_data = odmr.frequency_data if odmr else []
        signal_data = odmr.signal_data if odmr else {}

        # Initialize result
        result = {
            'point_index': point_idx,
            'target_position': position,
            'actual_position': actual_pos,
            'center_frequency': np.nan,
            'linewidth': np.nan,
            'splitting': np.nan,
            'n_features_found': 0,
            'fit_result': None,
        }

        # Get grid index for filename
        grid_idx = self._scan_data.point_index_to_grid_index(point_idx)
        if len(grid_idx) == 2:
            pixel_tag = f'pixel_x{grid_idx[0]:03d}_y{grid_idx[1]:03d}'
        else:
            pixel_tag = f'pixel_{point_idx:04d}'

        # Perform fitting if we have data and fit function
        if self._fit_function is not None and len(frequency_data) > 0:
            try:
                if signal_data:
                    channel_name = list(signal_data.keys())[0]
                    channel_data = signal_data[channel_name]

                    if isinstance(channel_data, list) and len(channel_data) > 0:
                        voltage_data = np.array(channel_data[0])
                        freq_data = np.array(frequency_data[0]) if isinstance(frequency_data, list) else np.array(frequency_data)
                    else:
                        voltage_data = np.array(channel_data)
                        freq_data = np.array(frequency_data)

                    # Determine if we should save fit plots
                    save_plot = self._save_odmr_fit_plots and self._current_scan_folder is not None
                    if save_plot:
                        # Create odmr_fits subfolder if it doesn't exist
                        fits_folder = os.path.join(self._current_scan_folder, 'odmr_fits')
                        os.makedirs(fits_folder, exist_ok=True)
                        fit_filename = os.path.join(fits_folder, pixel_tag)
                    else:
                        fit_filename = None

                    # Call fit_hyperfine with same parameters as sensitivity_sweep_logic
                    fit_result = self._fit_function(
                        freq_data,  # positional argument (like sensitivity_sweep)
                        voltage_data,  # positional argument (like sensitivity_sweep)
                        feature_prominence=self._fit_feature_prominence,
                        n_most_prominent_peaks=self._fit_n_most_prominent_peaks,
                        min_feature_height=self._fit_min_feature_height,
                        plot_result=False,
                        save_result_plot=save_plot,
                        filename=fit_filename
                    )

                    if fit_result is not None:
                        result['fit_result'] = fit_result
                        result['n_features_found'] = fit_result.get('n_features_found', 0)

                        zc_freqs = fit_result.get('zero_crossing_frequencies [Hz]', [])
                        linewidths = fit_result.get('linewidths [Hz]', [])

                        if isinstance(zc_freqs, np.ndarray) and len(zc_freqs) > 0:
                            valid_zc = zc_freqs[~np.isnan(zc_freqs)]
                            if len(valid_zc) > 0:
                                result['center_frequency'] = np.mean(valid_zc)
                                if len(valid_zc) >= 2:
                                    result['splitting'] = valid_zc[-1] - valid_zc[0]

                        if isinstance(linewidths, np.ndarray) and len(linewidths) > 0:
                            valid_lw = linewidths[~np.isnan(linewidths)]
                            if len(valid_lw) > 0:
                                result['linewidth'] = np.mean(valid_lw)
                        
                        self.log.debug(f"Point {point_idx}: center_freq={result['center_frequency']/1e9:.6f} GHz, "
                                      f"linewidth={result['linewidth']/1e3:.1f} kHz, "
                                      f"n_features={result['n_features_found']}")
                    else:
                        self.log.warning(f"Fit returned None at point {point_idx}")

            except Exception as e:
                self.log.warning(f"Fitting failed at point {point_idx}: {e}")

        # Fallback: extract basic stats from raw ODMR data if no fitting available
        if np.isnan(result['center_frequency']) and len(frequency_data) > 0 and signal_data:
            self.log.debug(f"Using fallback extraction for point {point_idx}")
            try:
                channel_name = list(signal_data.keys())[0]
                channel_data = signal_data[channel_name]

                if isinstance(channel_data, list) and len(channel_data) > 0:
                    voltage_data = np.array(channel_data[0])
                    freq_data = frequency_data[0] if isinstance(frequency_data, list) else frequency_data
                else:
                    voltage_data = np.array(channel_data)
                    freq_data = frequency_data

                freq_data = np.array(freq_data)

                min_idx = np.argmin(voltage_data)
                result['center_frequency'] = freq_data[min_idx]
                result['n_features_found'] = 1

                min_val = voltage_data[min_idx]
                max_val = np.max(voltage_data)
                half_max = (min_val + max_val) / 2

                above_half = voltage_data > half_max
                transitions = np.where(np.diff(above_half.astype(int)))[0]
                if len(transitions) >= 2:
                    left_idx = transitions[0]
                    right_idx = transitions[-1]
                    result['linewidth'] = abs(freq_data[right_idx] - freq_data[left_idx])

            except Exception as e:
                self.log.debug(f"Fallback extraction failed: {e}")

        # Update grid data
        if self._scan_data.is_2d:
            self._scan_data.center_frequency[grid_idx] = result['center_frequency']
            self._scan_data.linewidth[grid_idx] = result['linewidth']
            self._scan_data.splitting[grid_idx] = result['splitting']
            self._scan_data.fit_quality[grid_idx] = result['n_features_found']
            self._scan_data.odmr_fit_results[grid_idx[0]][grid_idx[1]] = result['fit_result']
        else:
            self._scan_data.center_frequency[grid_idx[0]] = result['center_frequency']
            self._scan_data.linewidth[grid_idx[0]] = result['linewidth']
            self._scan_data.splitting[grid_idx[0]] = result['splitting']
            self._scan_data.fit_quality[grid_idx[0]] = result['n_features_found']

        # Store raw ODMR data for this pixel (for later saving)
        if self._scan_data.odmr_raw_per_pixel is not None:
            try:
                # Extract and store raw ODMR frequency and signal data
                raw_odmr_dict = {
                    'target_position': position.copy(),
                    'actual_position': actual_pos.copy(),
                    'grid_index': grid_idx,
                    'frequency_data': None,
                    'signal_data': {},
                }
                
                if len(frequency_data) > 0:
                    # Store frequency data (handle list vs array)
                    if isinstance(frequency_data, list) and len(frequency_data) > 0:
                        raw_odmr_dict['frequency_data'] = np.array(frequency_data[0])
                    else:
                        raw_odmr_dict['frequency_data'] = np.array(frequency_data)
                
                if signal_data:
                    for ch_name, ch_data in signal_data.items():
                        if isinstance(ch_data, list) and len(ch_data) > 0:
                            raw_odmr_dict['signal_data'][ch_name] = np.array(ch_data[0])
                        else:
                            raw_odmr_dict['signal_data'][ch_name] = np.array(ch_data)
                
                self._scan_data.odmr_raw_per_pixel[point_idx] = raw_odmr_dict
                
            except Exception as e:
                self.log.debug(f"Failed to store raw ODMR data for point {point_idx}: {e}")

        return result

    def _collect_continuous_data_and_advance(self):
        """
        Collect streaming data at current position and advance to next point.
        
        Uses data buffered from the sigNewRawData signal, which allows non-blocking
        data collection. The time_series_logic continues to stream data to its GUI
        while we receive a copy via the signal.
        """
        motor = self._motor_hardware()
        point_idx = self._current_point_idx

        actual_pos = motor.get_pos()

        # Collect buffered data from the raw data buffer (populated by _on_new_raw_data)
        samples_collected = 0
        collected_data = {}
        
        for channel in self._scan_data.stream_data_mean.keys():
            if channel in self._ts_raw_data_buffer:
                # Get all buffered data for this channel and clear the buffer
                data_list = self._ts_raw_data_buffer[channel]
                collected_data[channel] = data_list.copy()
                samples_collected += len(data_list)
                # Clear the buffer for next position
                self._ts_raw_data_buffer[channel] = []
            else:
                collected_data[channel] = []

        # Update grid data
        grid_idx = self._scan_data.point_index_to_grid_index(point_idx)

        for channel, data_list in collected_data.items():
            if channel in self._scan_data.stream_data_raw:
                self._scan_data.stream_data_raw[channel][point_idx].extend(data_list)

                all_data = self._scan_data.stream_data_raw[channel][point_idx]
                if len(all_data) > 0:
                    mean_val = np.mean(all_data)
                    if self._scan_data.is_2d:
                        self._scan_data.stream_data_mean[channel][grid_idx] = mean_val
                    else:
                        self._scan_data.stream_data_mean[channel][grid_idx[0]] = mean_val

        result = {
            'point_index': point_idx,
            'target_position': self._current_pos_dict,
            'actual_position': actual_pos,
            'samples_collected': samples_collected,
        }

        self._advance_to_next_point(result)

    def _advance_to_next_point(self, result: Dict):
        """Advance to the next scan point after completing current one."""
        with self._thread_lock:
            # Store actual position in the scan data array
            actual_pos = result.get('actual_position', {})
            if actual_pos and self._scan_data.actual_positions is not None:
                point_idx = self._current_point_idx
                for i, axis in enumerate(self._scan_data.scan_axes):
                    if axis in actual_pos:
                        self._scan_data.actual_positions[point_idx, i] = actual_pos[axis]
            
            # Update progress
            self._scan_data.current_point_index += 1

            # Emit signals
            self.sigScanPointCompleted.emit(self._current_point_idx, result)
            self.sigScanDataUpdated.emit()

            # Schedule next point (via signal for non-blocking)
            self._sigNextPoint.emit()

    def _position_to_grid_index(self, position: Dict[str, float]) -> Optional[Tuple[int, ...]]:
        """
        Convert a motor position to grid indices.
        
        Args:
            position: Current motor position dict.
            
        Returns:
            Tuple of grid indices or None if outside scan range.
        """
        if self._scan_data is None:
            return None
        
        indices = []
        for i, (axis, (start, stop), resolution) in enumerate(
            zip(self._scan_data.scan_axes, self._scan_data.scan_range, 
                self._scan_data.scan_resolution)
        ):
            pos = position.get(axis, 0)
            
            # Check if within range
            if pos < min(start, stop) or pos > max(start, stop):
                return None
            
            # Calculate index (linear interpolation)
            if resolution > 1:
                idx = int((pos - start) / (stop - start) * (resolution - 1) + 0.5)
                idx = max(0, min(resolution - 1, idx))
            else:
                idx = 0
            
            indices.append(idx)
        
        return tuple(indices) if len(indices) == 2 else (indices[0],)
    
    def _finalize_scan(self, completed: bool):
        """
        Finalize the scan.

        Args:
            completed: True if scan finished normally, False if stopped.
        """
        # Clean up time series connection (CONTINUOUS_STREAM and CONTINUOUS_FREQ_TRACK modes)
        if self._scan_data.scan_mode in (ScanMode.CONTINUOUS_STREAM, ScanMode.CONTINUOUS_FREQ_TRACK):
            self._disconnect_time_series_signals()

            # Only stop time series reader if WE started it
            if self._ts_we_started:
                ts_logic = self._time_series_logic()
                if ts_logic is not None and hasattr(ts_logic, 'stop_reading'):
                    try:
                        ts_logic.stop_reading()
                    except Exception:
                        pass

            self._ts_raw_data_buffer = {}
            self._ts_we_started = False

        # Additional cleanup for CONTINUOUS_FREQ_TRACK mode
        if self._scan_data.scan_mode == ScanMode.CONTINUOUS_FREQ_TRACK:
            # Stop lock status polling
            if self._lock_status_timer is not None:
                self._lock_status_timer.stop()

            # Restore stream mode to 'demod' (error signal) for normal tracking GUI use
            tracking_logic = self._odmr_frequency_tracking_logic()
            if tracking_logic is not None:
                try:
                    tracking_logic.set_stream_mode('demod')
                    self.log.debug("Restored tracking stream mode to 'demod'")
                except Exception as e:
                    self.log.debug(f"Could not restore stream mode: {e}")

            # Reset tracking state
            self._paused_due_to_lock_loss = False

        # Update scan data
        self._scan_data.completed = completed
        self._scan_data.timestamp_end = datetime.datetime.now()
        self._scan_data.scan_duration = time.time() - self._scan_start_time

        # Reset state
        self._scan_state = ScanState.IDLE
        self._stop_requested = False
        
        if self.module_state() == 'locked':
            self.module_state.unlock()
        
        # Emit signals
        self.sigScanStateChanged.emit(self._scan_state)
        self.sigScanCompleted.emit(self._scan_data)
        
        status = "completed" if completed else "stopped"
        self.log.info(f"Scan {status}. Duration: {self._scan_data.scan_duration:.1f}s, "
                     f"Points: {self._scan_data.current_point_index}/{self._scan_data.total_points}")
    
    # =========================================================================
    # Data Saving
    # =========================================================================
    
    @QtCore.Slot(str)
    def save_scan_data(self, tag: str = None):
        """
        Save current scan data to file.
        
        Creates a dedicated folder for each XY scan containing all data files
        and subfolders (e.g., raw ODMR data per pixel).
        
        Folder structure:
            <module_data_dir>/YYYYMMDD-HHMM-SS_tag_motor_scan_MODE/
                center_frequency.dat
                center_frequency.pdf
                linewidth.dat
                ...
                odmr_raw_per_pixel/
                    pixel_x000_y000_odmr.dat
                    ...
        
        Args:
            tag: Optional tag to include in folder name.
        """
        with self._thread_lock:
            if self._scan_data is None:
                self.log.warning("No scan data to save.")
                return

            # Block save only if scan is actively running or stopping (not paused)
            # Allow saving when:
            # - Scan state is IDLE (no scan in progress)
            # - Scan state is PAUSED (data is stable, safe to save)
            if self._scan_state == ScanState.RUNNING:
                self.log.error('Unable to save scan data. Scan actively running. '
                               'Pause the scan first to save.')
                return
            if self._scan_state == ScanState.STOPPING:
                self.log.warning('Unable to save scan data. Scan is stopping, please wait.')
                return

            # Track if we're already locked (e.g., from a paused scan)
            # to avoid double-lock/unlock issues
            already_locked = (self.module_state() == 'locked')

            self.sigSaveStateChanged.emit(True)
            if not already_locked:
                self.module_state.lock()

            try:
                timestamp = datetime.datetime.now()
                timestamp_str = timestamp.strftime('%Y%m%d-%H%M-%S')
                
                # Use existing scan folder if available (created during scan for fit plots)
                # Otherwise create a new one
                if self._current_scan_folder is not None and os.path.isdir(self._current_scan_folder):
                    scan_folder = self._current_scan_folder
                    # Optionally rename to include tag
                    if tag:
                        old_folder = scan_folder
                        parent_dir = os.path.dirname(scan_folder)
                        old_name = os.path.basename(scan_folder)
                        # Insert tag after timestamp
                        parts = old_name.split('_', 1)
                        if len(parts) == 2:
                            new_name = f'{parts[0]}_{tag}_{parts[1]}'
                        else:
                            new_name = f'{old_name}_{tag}'
                        scan_folder = os.path.join(parent_dir, new_name)
                        if old_folder != scan_folder:
                            try:
                                os.rename(old_folder, scan_folder)
                                self._current_scan_folder = scan_folder
                            except OSError:
                                # If rename fails, use original folder
                                scan_folder = old_folder
                else:
                    # Build folder name following qudi convention: YYYYMMDD-HHMM-SS_nametag
                    nametag = f'{tag}_' if tag else ''
                    nametag += f'motor_scan_{self._scan_data.scan_mode.name}'
                    scan_folder_name = f'{timestamp_str}_{nametag}'
                    
                    # Create scan folder inside module_default_data_dir
                    scan_folder = os.path.join(self.module_default_data_dir, scan_folder_name)
                    os.makedirs(scan_folder, exist_ok=True)
                
                self.log.info(f"Saving scan data to {scan_folder}")
                
                # Create storage pointing to the scan folder
                data_storage = TextDataStorage(root_dir=scan_folder)
                
                # Prepare metadata
                metadata = {
                    'Scan Mode': self._scan_data.scan_mode.name,
                    'Scan Pattern': self._scan_data.scan_pattern.name,
                    'Scan Axes': str(self._scan_data.scan_axes),
                    'Scan Range': str(self._scan_data.scan_range),
                    'Scan Resolution': str(self._scan_data.scan_resolution),
                    'Total Points': self._scan_data.total_points,
                    'Completed Points': self._scan_data.current_point_index,
                    'Scan Duration (s)': self._scan_data.scan_duration,
                    'Completed': self._scan_data.completed,
                }
                
                # Add axis-specific metadata
                for i, axis in enumerate(self._scan_data.scan_axes):
                    metadata[f'{axis} axis min'] = self._scan_data.scan_range[i][0]
                    metadata[f'{axis} axis max'] = self._scan_data.scan_range[i][1]
                    metadata[f'{axis} axis resolution'] = self._scan_data.scan_resolution[i]
                
                file_path = None
                
                if self._scan_data.scan_mode == ScanMode.CONTINUOUS_STREAM:
                    # Save streaming data - one file per channel
                    if self._scan_data.stream_data_mean:
                        for channel, data in self._scan_data.stream_data_mean.items():
                            file_path, _, _ = data_storage.save_data(
                                data,
                                metadata=metadata,
                                nametag=channel,
                                timestamp=timestamp,
                                column_headers=f'{channel} data (columns is X, rows is Y)',
                                use_timestamp=False
                            )

                            # Save thumbnail if configured
                            if self._save_thumbnails and file_path:
                                fig = self._draw_figure(data, channel, unit='V')
                                fig_path = file_path.rsplit('.', 1)[0]
                                data_storage.save_thumbnail(fig, file_path=fig_path)
                                plt.close(fig)
                    else:
                        self.log.warning("No stream data to save.")

                elif self._scan_data.scan_mode == ScanMode.CONTINUOUS_FREQ_TRACK:
                    # Save absolute frequency data
                    # Add frequency tracking metadata
                    metadata['Zero-crossing History'] = str(self._tracking_zero_crossing_history)
                    if self._tracking_zero_crossing is not None:
                        metadata['Final Zero-crossing (Hz)'] = self._tracking_zero_crossing

                    if self._scan_data.stream_data_mean:
                        for channel, data in self._scan_data.stream_data_mean.items():
                            # Determine unit based on channel name
                            if channel == 'absolute_frequency':
                                unit = 'Hz'
                                header = 'Absolute Frequency (Hz) (columns is X, rows is Y)'
                            else:
                                unit = ''
                                header = f'{channel} data (columns is X, rows is Y)'

                            file_path, _, _ = data_storage.save_data(
                                data,
                                metadata=metadata,
                                nametag=channel,
                                timestamp=timestamp,
                                column_headers=header,
                                use_timestamp=False
                            )

                            # Save thumbnail - display in GHz for readability
                            if self._save_thumbnails and file_path:
                                if channel == 'absolute_frequency':
                                    # Convert Hz to GHz for display
                                    data_ghz = data / 1e9
                                    fig = self._draw_figure(data_ghz, 'Absolute Frequency', unit='GHz')
                                else:
                                    fig = self._draw_figure(data, channel, unit=unit)
                                fig_path = file_path.rsplit('.', 1)[0]
                                data_storage.save_thumbnail(fig, file_path=fig_path)
                                plt.close(fig)
                    else:
                        self.log.warning("No frequency tracking data to save.")

                elif self._scan_data.scan_mode == ScanMode.STEP_ODMR:
                    # Save ODMR fit result arrays
                    if self._scan_data.center_frequency is not None:
                        file_path, _, _ = data_storage.save_data(
                            self._scan_data.center_frequency,
                            metadata=metadata,
                            nametag='center_frequency',
                            timestamp=timestamp,
                            column_headers='Center Frequency (Hz) (columns is X, rows is Y)',
                            use_timestamp=False
                        )
                        
                        if self._save_thumbnails and file_path:
                            fig = self._draw_figure(
                                self._scan_data.center_frequency, 
                                'Center Frequency', 
                                unit='Hz'
                            )
                            fig_path = file_path.rsplit('.', 1)[0]
                            data_storage.save_thumbnail(fig, file_path=fig_path)
                            plt.close(fig)
                        
                    if self._scan_data.linewidth is not None:
                        file_path, _, _ = data_storage.save_data(
                            self._scan_data.linewidth,
                            metadata=metadata,
                            nametag='linewidth',
                            timestamp=timestamp,
                            column_headers='Linewidth (Hz) (columns is X, rows is Y)',
                            use_timestamp=False
                        )
                        
                        if self._save_thumbnails and file_path:
                            fig = self._draw_figure(
                                self._scan_data.linewidth,
                                'Linewidth',
                                unit='Hz'
                            )
                            fig_path = file_path.rsplit('.', 1)[0]
                            data_storage.save_thumbnail(fig, file_path=fig_path)
                            plt.close(fig)
                        
                    if self._scan_data.splitting is not None:
                        file_path, _, _ = data_storage.save_data(
                            self._scan_data.splitting,
                            metadata=metadata,
                            nametag='splitting',
                            timestamp=timestamp,
                            column_headers='Splitting (Hz) (columns is X, rows is Y)',
                            use_timestamp=False
                        )
                        
                        if self._save_thumbnails and file_path:
                            fig = self._draw_figure(
                                self._scan_data.splitting,
                                'Splitting',
                                unit='Hz'
                            )
                            fig_path = file_path.rsplit('.', 1)[0]
                            data_storage.save_thumbnail(fig, file_path=fig_path)
                            plt.close(fig)
                        
                    if self._scan_data.fit_quality is not None:
                        file_path, _, _ = data_storage.save_data(
                            self._scan_data.fit_quality,
                            metadata=metadata,
                            nametag='fit_quality',
                            timestamp=timestamp,
                            column_headers='Fit Quality (columns is X, rows is Y)',
                            use_timestamp=False
                        )
                        
                        if self._save_thumbnails and file_path:
                            fig = self._draw_figure(
                                self._scan_data.fit_quality,
                                'Fit Quality',
                                unit=''
                            )
                            fig_path = file_path.rsplit('.', 1)[0]
                            data_storage.save_thumbnail(fig, file_path=fig_path)
                            plt.close(fig)
                    
                    # Save raw ODMR scans per pixel in a subfolder
                    if self._scan_data.odmr_raw_per_pixel is not None:
                        self._save_odmr_raw_per_pixel(
                            scan_folder, 
                            timestamp, 
                            metadata
                        )
                
                # Save positions data (for all modes)
                self._save_positions_data(scan_folder, timestamp, metadata)
                    
                self.log.info(f"Scan data saved to: {scan_folder}")

            finally:
                # Only unlock if we locked it ourselves
                if not already_locked:
                    self.module_state.unlock()
                self.sigSaveStateChanged.emit(False)
    
    def _draw_figure(self, data: np.ndarray, data_label: str, unit: str = '') -> plt.Figure:
        """
        Draw a 2D color map figure of the scan data.
        
        Args:
            data: 2D numpy array with scan data (shape: nx, ny)
            data_label: Label for the data (e.g., 'Center Frequency')
            unit: Unit string for the data (e.g., 'Hz')
            
        Returns:
            matplotlib.figure.Figure: Figure object ready for saving.
        """
        if self._scan_data is None:
            fig, ax = plt.subplots()
            ax.text(0.5, 0.5, 'No data', ha='center', va='center')
            return fig
        
        # Get scan ranges
        x_range = self._scan_data.scan_range[0]
        y_range = self._scan_data.scan_range[1] if len(self._scan_data.scan_range) > 1 else (0, 1)
        
        # Handle colorbar range - ignore NaN values
        valid_data = data[~np.isnan(data)]
        if len(valid_data) == 0:
            cbar_range = (0, 1)
        else:
            cbar_range = (np.nanmin(data), np.nanmax(data))
        
        # Calculate SI scaling for axes
        si_prefix_x = ScaledFloat(x_range[1] - x_range[0]).scale
        si_factor_x = ScaledFloat(x_range[1] - x_range[0]).scale_val
        si_prefix_y = ScaledFloat(y_range[1] - y_range[0]).scale
        si_factor_y = ScaledFloat(y_range[1] - y_range[0]).scale_val
        
        # Calculate SI scaling for colorbar
        if cbar_range[1] != cbar_range[0]:
            si_prefix_cb = ScaledFloat(cbar_range[1] - cbar_range[0]).scale
            si_factor_cb = ScaledFloat(cbar_range[1] - cbar_range[0]).scale_val
        else:
            si_prefix_cb = ScaledFloat(cbar_range[1]).scale if cbar_range[1] != 0 else ''
            si_factor_cb = ScaledFloat(cbar_range[1]).scale_val if cbar_range[1] != 0 else 1
        
        # Create figure
        fig, ax = plt.subplots()
        
        # Create image plot
        # Data shape is (nx, ny), but imshow expects (rows, cols) = (ny, nx)
        # So we transpose the data for correct display
        cfimage = ax.imshow(
            data.T / si_factor_cb,
            cmap='inferno',
            origin='lower',
            vmin=cbar_range[0] / si_factor_cb,
            vmax=cbar_range[1] / si_factor_cb,
            interpolation='none',
            extent=(
                x_range[0] / si_factor_x,
                x_range[1] / si_factor_x,
                y_range[0] / si_factor_y,
                y_range[1] / si_factor_y
            )
        )
        
        # Set axis labels
        x_axis_name = self._scan_data.scan_axes[0] if self._scan_data.scan_axes else 'x'
        y_axis_name = self._scan_data.scan_axes[1] if len(self._scan_data.scan_axes) > 1 else 'y'
        ax.set_xlabel(f'{x_axis_name} position ({si_prefix_x}m)')
        ax.set_ylabel(f'{y_axis_name} position ({si_prefix_y}m)')
        
        # Configure axis appearance (use 1 for aspect ratio, consistent with scanning_data_logic)
        ax.set_aspect(1)
        ax.spines['bottom'].set_position(('outward', 10))
        ax.spines['left'].set_position(('outward', 10))
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()
        
        # Draw the colorbar
        cbar = plt.colorbar(cfimage, shrink=0.8)
        if unit:
            cbar.set_label(f'{data_label} ({si_prefix_cb}{unit})')
        else:
            cbar.set_label(f'{data_label}')
        
        # Remove ticks from colorbar for cleaner image
        cbar.ax.tick_params(which='both', length=0)
        
        # Add scan metadata annotation
        metainfo_str = self._get_figure_metadata_string()
        if metainfo_str:
            ax.annotate(
                metainfo_str,
                xy=(1.10, -0.17),
                xycoords='axes fraction',
                horizontalalignment='left',
                verticalalignment='bottom',
                fontsize=7,
                color='grey'
            )
        
        return fig
    
    def _get_figure_metadata_string(self) -> str:
        """
        Generate a metadata string for the figure annotation.
        
        Returns:
            Formatted string with scan metadata.
        """
        if self._scan_data is None:
            return ''
        
        lines = []
        
        # Scan mode and pattern
        lines.append(f"Mode: {self._scan_data.scan_mode.name}")
        lines.append(f"Pattern: {self._scan_data.scan_pattern.name}")
        
        # Resolution
        if self._scan_data.scan_resolution:
            res_str = ' x '.join(str(r) for r in self._scan_data.scan_resolution)
            lines.append(f"Resolution: {res_str} points")
        
        # Duration
        if self._scan_data.scan_duration > 0:
            lines.append(f"Duration: {self._scan_data.scan_duration:.1f}s")
        
        return '\n'.join(lines)
    
    def _save_odmr_raw_per_pixel(
        self, 
        scan_folder: str, 
        timestamp: datetime.datetime,
        metadata: Dict
    ):
        """
        Save raw ODMR scans for each pixel in a subfolder.
        
        Creates a subfolder 'odmr_raw_per_pixel' inside the scan folder containing
        individual ODMR data files for each pixel, enabling detailed post-analysis.
        
        Args:
            scan_folder: Path to the main scan folder
            timestamp: Timestamp of the save operation
            metadata: Base metadata dict
        """
        if self._scan_data is None or self._scan_data.odmr_raw_per_pixel is None:
            return
        
        # Count how many pixels have data
        valid_pixels = [p for p in self._scan_data.odmr_raw_per_pixel if p is not None]
        if not valid_pixels:
            return
        
        try:
            # Create subfolder inside the scan folder
            odmr_subfolder = os.path.join(scan_folder, 'odmr_raw_per_pixel')
            os.makedirs(odmr_subfolder, exist_ok=True)
            
            for point_idx, pixel_data in enumerate(self._scan_data.odmr_raw_per_pixel):
                if pixel_data is None:
                    continue
                
                grid_idx = pixel_data.get('grid_index', (point_idx,))
                target_pos = pixel_data.get('target_position', {})
                actual_pos = pixel_data.get('actual_position', {})
                freq_data = pixel_data.get('frequency_data')
                signal_data = pixel_data.get('signal_data', {})
                
                if freq_data is None or len(signal_data) == 0:
                    continue
                
                # Create pixel-specific metadata
                pixel_metadata = metadata.copy()
                pixel_metadata['Pixel Index'] = point_idx
                pixel_metadata['Grid Index'] = str(grid_idx)
                for axis, val in target_pos.items():
                    pixel_metadata[f'Target {axis} (m)'] = val
                for axis, val in actual_pos.items():
                    pixel_metadata[f'Actual {axis} (m)'] = val
                
                # Create filename with grid indices for easy sorting
                if len(grid_idx) == 2:
                    pixel_tag = f'pixel_x{grid_idx[0]:03d}_y{grid_idx[1]:03d}'
                else:
                    pixel_tag = f'pixel_{point_idx:04d}'
                
                # Build data array: frequency column + signal columns
                n_points = len(freq_data)
                columns = [freq_data]
                col_headers = ['Frequency (Hz)']
                
                for ch_name, ch_data in signal_data.items():
                    if len(ch_data) == n_points:
                        columns.append(ch_data)
                        col_headers.append(f'{ch_name} (V)')
                
                # Stack columns into 2D array
                data_array = np.column_stack(columns)
                
                # Save to file
                file_path = os.path.join(odmr_subfolder, f'{pixel_tag}_odmr.dat')
                
                # Write file manually with header
                with open(file_path, 'w') as f:
                    # Write metadata header
                    f.write('# ODMR Raw Data for Motor Scan Pixel\n')
                    f.write(f'# Saved: {timestamp.isoformat()}\n')
                    f.write('#\n')
                    for key, val in pixel_metadata.items():
                        f.write(f'# {key}: {val}\n')
                    f.write('#\n')
                    f.write('# ' + '\t'.join(col_headers) + '\n')
                    
                    # Write data
                    for row in data_array:
                        f.write('\t'.join(f'{v:.15e}' for v in row) + '\n')
            
        except Exception as e:
            self.log.error(f"Failed to save raw ODMR data per pixel: {e}")

    def _save_positions_data(
        self,
        scan_folder: str,
        timestamp: datetime.datetime,
        metadata: Dict
    ):
        """
        Save target and actual positions for all scan points.
        
        Creates a 'positions.dat' file in the scan folder containing a table with:
        - Point index and grid indices
        - Target positions for each axis
        - Actual (measured) positions for each axis
        - Position errors (actual - target) for each axis
        
        This enables post-scan analysis of positioning accuracy.
        
        Args:
            scan_folder: Path to the main scan folder
            timestamp: Timestamp of the save operation
            metadata: Base metadata dict
        """
        if self._scan_data is None:
            return
        
        if self._scan_data.target_positions is None:
            return
        
        try:
            file_path = os.path.join(scan_folder, 'positions.dat')
            axes = self._scan_data.scan_axes
            n_axes = len(axes)
            n_points = self._scan_data.total_points
            
            # Build column headers
            col_headers = ['Point_Index']
            if self._scan_data.is_2d:
                col_headers.extend(['Grid_X', 'Grid_Y'])
            else:
                col_headers.append('Grid_Index')
            
            for axis in axes:
                col_headers.append(f'Target_{axis} (m)')
            for axis in axes:
                col_headers.append(f'Actual_{axis} (m)')
            for axis in axes:
                col_headers.append(f'Error_{axis} (m)')
            
            with open(file_path, 'w') as f:
                # Write header
                f.write('# Motor Scan Position Data\n')
                f.write(f'# Saved: {timestamp.isoformat()}\n')
                f.write('#\n')
                for key, val in metadata.items():
                    f.write(f'# {key}: {val}\n')
                f.write('#\n')
                
                # Compute position statistics
                if self._scan_data.actual_positions is not None:
                    valid_mask = ~np.isnan(self._scan_data.actual_positions).any(axis=1)
                    if valid_mask.any():
                        errors = self._scan_data.actual_positions[valid_mask] - self._scan_data.target_positions[valid_mask]
                        mean_error = np.mean(np.abs(errors), axis=0)
                        max_error = np.max(np.abs(errors), axis=0)
                        rms_error = np.sqrt(np.mean(errors**2, axis=0))
                        
                        f.write('# Position Statistics:\n')
                        for i, axis in enumerate(axes):
                            f.write(f'#   {axis}-axis: mean_abs_error={mean_error[i]*1e6:.2f}um, '
                                    f'max_abs_error={max_error[i]*1e6:.2f}um, '
                                    f'rms_error={rms_error[i]*1e6:.2f}um\n')
                        f.write('#\n')
                
                # Write column headers
                f.write('# ' + '\t'.join(col_headers) + '\n')
                
                # Write data rows
                for point_idx in range(n_points):
                    row = [str(point_idx)]
                    
                    # Grid indices
                    grid_idx = self._scan_data.point_index_to_grid_index(point_idx)
                    if self._scan_data.is_2d:
                        row.extend([str(grid_idx[0]), str(grid_idx[1])])
                    else:
                        row.append(str(grid_idx[0]))
                    
                    # Target positions
                    target = self._scan_data.target_positions[point_idx]
                    for i in range(n_axes):
                        row.append(f'{target[i]:.9e}')
                    
                    # Actual positions
                    if self._scan_data.actual_positions is not None:
                        actual = self._scan_data.actual_positions[point_idx]
                        for i in range(n_axes):
                            if np.isnan(actual[i]):
                                row.append('nan')
                            else:
                                row.append(f'{actual[i]:.9e}')
                        
                        # Errors
                        for i in range(n_axes):
                            if np.isnan(actual[i]):
                                row.append('nan')
                            else:
                                error = actual[i] - target[i]
                                row.append(f'{error:.9e}')
                    else:
                        # No actual positions recorded
                        for i in range(n_axes):
                            row.append('nan')
                        for i in range(n_axes):
                            row.append('nan')
                    
                    f.write('\t'.join(row) + '\n')
            
            self.log.debug(f"Saved positions data to {file_path}")
            
        except Exception as e:
            self.log.error(f"Failed to save positions data: {e}")
