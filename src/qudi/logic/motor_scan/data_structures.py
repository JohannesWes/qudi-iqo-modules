# -*- coding: utf-8 -*-
"""
Data structures for motor-based XY scanning.

Contains enums for scan mode, pattern, and state, as well as the MotorScanData
dataclass for storing scan configuration and results.
"""

import datetime
import numpy as np
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Tuple, Optional, Any


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
        
        if self.scan_mode in (ScanMode.CONTINUOUS_STREAM, ScanMode.CONTINUOUS_FREQ_TRACK):
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
