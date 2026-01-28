# -*- coding: utf-8 -*-
"""
Data loading mixin for MotorScanLogic.

Handles loading previously saved STEP_ODMR scan data from files for review
in the GUI with full pixel-by-pixel ODMR spectrum viewing capability.
"""

import os
import re
import glob
import datetime
import numpy as np
from typing import Dict, List, Tuple, Optional, Any

from PySide2 import QtCore

from .data_structures import ScanMode, ScanPattern, MotorScanData


class DataLoadingMixin:
    """
    Mixin class providing data loading functionality for MotorScanLogic.

    This mixin provides:
    - Loading saved STEP_ODMR scan data from a folder
    - Reconstructing MotorScanData objects from saved files
    - Parsing .dat file headers (TextDataStorage format)
    - Loading per-pixel ODMR raw data

    The parent class must provide:
    - _thread_lock: RecursiveMutex for thread safety
    - _scan_state: Current ScanState
    - sigScanDataUpdated: Signal()
    - sigLoadedDataChanged: Signal(bool)
    - log: Logger instance
    """

    def _init_data_loading(self):
        """
        Initialize data loading attributes.

        Call this from on_activate() in the parent class.
        """
        self._loaded_scan_data: Optional[MotorScanData] = None
        self._viewing_loaded_data: bool = False
        self._loaded_data_folder: Optional[str] = None

    @QtCore.Slot(str)
    def load_scan_data_from_folder(self, folder_path: str) -> bool:
        """
        Load STEP_ODMR scan data from a saved folder.

        Validates the folder contents, parses all data files, and reconstructs
        a MotorScanData object for display in the GUI.

        Args:
            folder_path: Path to the scan folder containing .dat files.

        Returns:
            True if load succeeded, False otherwise.
        """
        from .data_structures import ScanState

        with self._thread_lock:
            # Check if scan is in progress
            if self._scan_state not in (ScanState.IDLE,):
                self.log.error("Cannot load data while scan is in progress.")
                return False

            # Validate folder
            is_valid, error_msg = self._validate_scan_folder(folder_path)
            if not is_valid:
                self.log.error(f"Invalid scan folder: {error_msg}")
                return False

            try:
                # Reconstruct scan data from files
                loaded_data = self._reconstruct_scan_data(folder_path)

                if loaded_data is None:
                    self.log.error("Failed to reconstruct scan data from folder.")
                    return False

                # Store loaded data
                self._loaded_scan_data = loaded_data
                self._viewing_loaded_data = True
                self._loaded_data_folder = folder_path

                # Emit signals to update GUI
                self.sigLoadedDataChanged.emit(True)
                self.sigScanDataUpdated.emit()

                self.log.info(f"Successfully loaded scan data from: {folder_path}")
                return True

            except Exception as e:
                self.log.error(f"Failed to load scan data: {e}", exc_info=True)
                return False

    @QtCore.Slot()
    def clear_loaded_data(self):
        """
        Clear loaded data and return to viewing current scan data.
        """
        with self._thread_lock:
            self._loaded_scan_data = None
            self._viewing_loaded_data = False
            self._loaded_data_folder = None

            self.sigLoadedDataChanged.emit(False)
            self.sigScanDataUpdated.emit()

            self.log.debug("Cleared loaded scan data.")

    def _validate_scan_folder(self, folder_path: str) -> Tuple[bool, str]:
        """
        Validate that a folder contains valid STEP_ODMR scan data.

        Args:
            folder_path: Path to the scan folder.

        Returns:
            Tuple of (is_valid, error_message).
        """
        if not os.path.isdir(folder_path):
            return False, f"Not a directory: {folder_path}"

        # Check for required file (center_frequency.dat)
        cf_path = os.path.join(folder_path, 'center_frequency.dat')
        if not os.path.isfile(cf_path):
            return False, "Missing required file: center_frequency.dat"

        # Parse header to check scan mode
        try:
            metadata = self._parse_dat_file_header(cf_path)
        except Exception as e:
            return False, f"Failed to parse header: {e}"

        # Check scan mode
        scan_mode = metadata.get('scan mode', '').strip("'\"")
        if scan_mode != 'STEP_ODMR':
            return False, f"Invalid scan mode: {scan_mode}. Only STEP_ODMR is supported for loading."

        return True, ""

    def _reconstruct_scan_data(self, folder_path: str) -> Optional[MotorScanData]:
        """
        Reconstruct a MotorScanData object from saved files.

        Args:
            folder_path: Path to the scan folder.

        Returns:
            Reconstructed MotorScanData object, or None on failure.
        """
        # Parse metadata from center_frequency.dat
        cf_path = os.path.join(folder_path, 'center_frequency.dat')
        metadata = self._parse_dat_file_header(cf_path)

        # Extract scan configuration
        scan_mode = ScanMode[metadata.get('scan mode', 'STEP_ODMR').strip("'\"")]
        scan_pattern = ScanPattern[metadata.get('scan pattern', 'SNAKE_X').strip("'\"")]

        # Parse tuple strings safely
        scan_axes = self._parse_tuple_string(metadata.get('scan axes', "('x', 'y')"))
        scan_range = self._parse_tuple_string(metadata.get('scan range', "((0.0, 0.01), (0.0, 0.01))"))
        scan_resolution = self._parse_tuple_string(metadata.get('scan resolution', "(10, 10)"))

        # Parse other metadata
        total_points = int(metadata.get('total points', 0))
        completed_points = int(metadata.get('completed points', 0))
        scan_duration = float(metadata.get('scan duration (s)', 0.0))
        completed_str = metadata.get('completed', 'False').strip().strip("'\"")
        completed = completed_str.lower() == 'true'

        # Parse timestamp
        timestamp_str = metadata.get('timestamp', '')
        timestamp_start = None
        if timestamp_str:
            try:
                timestamp_start = datetime.datetime.fromisoformat(timestamp_str)
            except ValueError:
                pass

        # Validate parsed values
        if not isinstance(scan_axes, (list, tuple)):
            self.log.warning(f"Could not parse scan_axes: {scan_axes}, using default")
            scan_axes = ('x', 'y')
        if not isinstance(scan_range, (list, tuple)):
            self.log.warning(f"Could not parse scan_range: {scan_range}, using default")
            scan_range = ((0.0, 0.01), (0.0, 0.01))
        if not isinstance(scan_resolution, (list, tuple)):
            self.log.warning(f"Could not parse scan_resolution: {scan_resolution}, using default")
            scan_resolution = (10, 10)

        # Create MotorScanData instance
        scan_data = MotorScanData(
            scan_axes=tuple(scan_axes),
            scan_range=tuple(tuple(r) if isinstance(r, (list, tuple)) else (r, r) for r in scan_range),
            scan_resolution=tuple(int(r) for r in scan_resolution),
            scan_mode=scan_mode,
            scan_pattern=scan_pattern,
            timestamp_start=timestamp_start,
            scan_duration=scan_duration,
            completed=completed,
        )

        scan_data.total_points = total_points
        scan_data.current_point_index = completed_points

        # Load 2D arrays
        scan_data.center_frequency = self._load_2d_array(cf_path)

        linewidth_path = os.path.join(folder_path, 'linewidth.dat')
        if os.path.isfile(linewidth_path):
            scan_data.linewidth = self._load_2d_array(linewidth_path)

        splitting_path = os.path.join(folder_path, 'splitting.dat')
        if os.path.isfile(splitting_path):
            scan_data.splitting = self._load_2d_array(splitting_path)

        fit_quality_path = os.path.join(folder_path, 'fit_quality.dat')
        if os.path.isfile(fit_quality_path):
            scan_data.fit_quality = self._load_2d_array(fit_quality_path)

        # Load per-pixel ODMR raw data
        odmr_raw_folder = os.path.join(folder_path, 'odmr_raw_per_pixel')
        if os.path.isdir(odmr_raw_folder):
            scan_data.odmr_raw_per_pixel = self._load_odmr_raw_files(odmr_raw_folder, scan_data)

            # Also create odmr_fit_results structure for compatibility
            # (even though we don't have the actual fit result dicts)
            if scan_data.is_2d:
                nx, ny = scan_data.scan_resolution
                scan_data.odmr_fit_results = [[None for _ in range(ny)] for _ in range(nx)]

                # Populate with basic info from loaded center_frequency, linewidth, etc.
                if scan_data.center_frequency is not None:
                    for ix in range(nx):
                        for iy in range(ny):
                            cf = scan_data.center_frequency[ix, iy]
                            lw = scan_data.linewidth[ix, iy] if scan_data.linewidth is not None else np.nan
                            sp = scan_data.splitting[ix, iy] if scan_data.splitting is not None else np.nan
                            fq = scan_data.fit_quality[ix, iy] if scan_data.fit_quality is not None else np.nan

                            if not np.isnan(cf):
                                # Create a minimal fit result dict for display
                                scan_data.odmr_fit_results[ix][iy] = {
                                    'zero_crossing_frequencies [Hz]': [cf],
                                    'linewidths [Hz]': [lw] if not np.isnan(lw) else [],
                                    'n_features_found': int(fq) if not np.isnan(fq) else 1,
                                }

        # Generate target positions grid
        scan_data.target_positions = scan_data.get_flat_target_positions()

        return scan_data

    def _parse_dat_file_header(self, file_path: str) -> Dict[str, str]:
        """
        Parse the header of a .dat file.

        Handles both TextDataStorage format (# [Section], # key=value) and
        simple format (# key: value).

        Args:
            file_path: Path to the .dat file.

        Returns:
            Dictionary of metadata key-value pairs.
        """
        metadata = {}

        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                # Stop at end of header marker or first data line
                if not line.startswith('#'):
                    break
                if '---- END HEADER ----' in line:
                    break

                line = line[1:].strip()  # Remove leading '#' and whitespace

                # Skip section headers and empty lines
                if not line or line.startswith('[') or line == '#':
                    continue

                # Parse key=value format (TextDataStorage)
                if '=' in line:
                    key, _, value = line.partition('=')
                    key = key.strip().lower()
                    value = value.strip()
                    if key and value:
                        metadata[key] = value

                # Parse key: value format (simple header)
                elif ':' in line:
                    key, _, value = line.partition(':')
                    key = key.strip().lower()
                    value = value.strip()
                    if key and value:
                        metadata[key] = value

        return metadata

    def _parse_tuple_string(self, s: str) -> Any:
        """
        Safely parse a string representation of a tuple/list.

        Args:
            s: String like "('x', 'y')" or "((0.0, 0.01), (0.0, 0.01))"
               May have surrounding quotes from TextDataStorage format.

        Returns:
            Parsed tuple/list, or the original string on failure.
        """
        s = s.strip()
        if not s:
            return None

        # Remove surrounding quotes if present (TextDataStorage format)
        # e.g., "'(40, 40)'" -> "(40, 40)"
        if (s.startswith("'") and s.endswith("'")) or (s.startswith('"') and s.endswith('"')):
            s = s[1:-1]

        try:
            # Use ast.literal_eval for safe parsing
            import ast
            return ast.literal_eval(s)
        except (ValueError, SyntaxError):
            return s

    def _load_2d_array(self, file_path: str) -> Optional[np.ndarray]:
        """
        Load a 2D numpy array from a .dat file.

        The internal array convention is array[ix, iy] with shape (nx, ny).
        TextDataStorage saves arrays by iterating over the first dimension,
        so files have nx rows and ny columns. No transpose is needed.

        Args:
            file_path: Path to the .dat file.

        Returns:
            2D numpy array, or None on failure.
        """
        try:
            # Load data, skipping header lines (marked with #)
            data = np.loadtxt(file_path, comments='#', delimiter='\t')

            # Data is stored with shape (nx, ny) - no transpose needed
            return data

        except Exception as e:
            self.log.warning(f"Failed to load 2D array from {file_path}: {e}")
            return None

    def _load_odmr_raw_files(
        self,
        odmr_folder: str,
        scan_data: MotorScanData
    ) -> List[Optional[Dict]]:
        """
        Load all ODMR raw data files from the odmr_raw_per_pixel subfolder.

        Args:
            odmr_folder: Path to the odmr_raw_per_pixel folder.
            scan_data: MotorScanData object (for scan configuration).

        Returns:
            List of dicts with ODMR data per point (indexed by flat point index).
        """
        total_points = scan_data.total_points
        odmr_raw_per_pixel = [None for _ in range(total_points)]

        # Find all pixel files
        pattern = os.path.join(odmr_folder, 'pixel_x*_y*_odmr.dat')
        pixel_files = glob.glob(pattern)

        self.log.debug(f"Found {len(pixel_files)} ODMR raw files in {odmr_folder}")

        for file_path in pixel_files:
            try:
                # Parse filename to get grid indices
                filename = os.path.basename(file_path)
                match = re.match(r'pixel_x(\d+)_y(\d+)_odmr\.dat', filename)
                if not match:
                    continue

                ix = int(match.group(1))
                iy = int(match.group(2))

                # Convert grid index to flat index
                flat_idx = self._grid_index_to_flat_index(ix, iy, scan_data)
                if flat_idx is None or flat_idx >= total_points:
                    continue

                # Parse header for metadata
                header = self._parse_dat_file_header(file_path)

                # Parse target/actual positions from header
                target_position = {}
                actual_position = {}
                for axis in scan_data.scan_axes:
                    target_key = f'target {axis} (m)'
                    actual_key = f'actual {axis} (m)'
                    if target_key in header:
                        try:
                            target_position[axis] = float(header[target_key])
                        except ValueError:
                            pass
                    if actual_key in header:
                        try:
                            actual_position[axis] = float(header[actual_key])
                        except ValueError:
                            pass

                # Load data columns
                data = np.loadtxt(file_path, comments='#', delimiter='\t')

                if data.ndim == 1:
                    # Single column - just frequency? Skip
                    continue

                # First column is frequency, rest are signal channels
                frequency_data = data[:, 0]

                # Parse column headers to get channel names
                signal_data = {}
                col_headers = self._get_column_headers(file_path)

                for col_idx in range(1, data.shape[1]):
                    if col_idx < len(col_headers):
                        # Extract channel name from header like "ch1 (V)"
                        ch_header = col_headers[col_idx]
                        ch_name = ch_header.split('(')[0].strip()
                    else:
                        ch_name = f'ch{col_idx}'
                    signal_data[ch_name] = data[:, col_idx]

                # Store in list
                odmr_raw_per_pixel[flat_idx] = {
                    'grid_index': (ix, iy),
                    'target_position': target_position,
                    'actual_position': actual_position,
                    'frequency_data': frequency_data,
                    'signal_data': signal_data,
                }

            except Exception as e:
                self.log.warning(f"Failed to load ODMR file {file_path}: {e}")
                continue

        loaded_count = sum(1 for p in odmr_raw_per_pixel if p is not None)
        self.log.debug(f"Loaded {loaded_count}/{total_points} ODMR raw data files")

        return odmr_raw_per_pixel

    def _grid_index_to_flat_index(
        self,
        ix: int,
        iy: int,
        scan_data: MotorScanData
    ) -> Optional[int]:
        """
        Convert grid indices (ix, iy) to flat point index.

        Accounts for scan pattern (LINE_BY_LINE vs SNAKE).

        Args:
            ix: X grid index
            iy: Y grid index
            scan_data: MotorScanData with scan configuration

        Returns:
            Flat point index, or None if invalid.
        """
        if not scan_data.is_2d:
            return ix

        nx, ny = scan_data.scan_resolution

        if ix < 0 or ix >= nx or iy < 0 or iy >= ny:
            return None

        pattern = scan_data.scan_pattern

        if pattern in (ScanPattern.LINE_BY_LINE_X, ScanPattern.SNAKE_X):
            # Fast axis is X: flat_index = iy * nx + ix_in_line
            if pattern == ScanPattern.SNAKE_X and iy % 2 == 1:
                # Reverse X for odd lines
                ix_in_line = nx - 1 - ix
            else:
                ix_in_line = ix
            return iy * nx + ix_in_line

        elif pattern in (ScanPattern.LINE_BY_LINE_Y, ScanPattern.SNAKE_Y):
            # Fast axis is Y: flat_index = ix * ny + iy_in_line
            if pattern == ScanPattern.SNAKE_Y and ix % 2 == 1:
                # Reverse Y for odd lines
                iy_in_line = ny - 1 - iy
            else:
                iy_in_line = iy
            return ix * ny + iy_in_line

        return None

    def _get_column_headers(self, file_path: str) -> List[str]:
        """
        Extract column headers from the last comment line before data.

        Args:
            file_path: Path to the .dat file.

        Returns:
            List of column header strings.
        """
        headers = []
        last_comment_line = ''

        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('#'):
                    last_comment_line = line[1:].strip()
                else:
                    break

        # Split by tab to get column headers
        if last_comment_line:
            headers = last_comment_line.split('\t')

        return headers

    @property
    def loaded_scan_data(self) -> Optional[MotorScanData]:
        """Return the loaded scan data (separate from current scan data)."""
        with self._thread_lock:
            return self._loaded_scan_data

    @property
    def is_viewing_loaded_data(self) -> bool:
        """Return True if viewing loaded data instead of current scan."""
        with self._thread_lock:
            return self._viewing_loaded_data

    @property
    def loaded_data_folder(self) -> Optional[str]:
        """Return the path to the loaded data folder."""
        with self._thread_lock:
            return self._loaded_data_folder

    @property
    def display_scan_data(self) -> Optional[MotorScanData]:
        """
        Return the scan data to display in the GUI.

        Returns loaded data if viewing loaded, otherwise returns current scan data.
        """
        with self._thread_lock:
            if self._viewing_loaded_data and self._loaded_scan_data is not None:
                return self._loaded_scan_data
            return self._scan_data
