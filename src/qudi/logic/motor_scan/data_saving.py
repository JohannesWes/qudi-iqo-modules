# -*- coding: utf-8 -*-
"""
Data saving mixin for MotorScanLogic.

Handles saving scan data to files and generating figures.
"""

import os
import datetime
import numpy as np
from typing import Dict

import matplotlib.pyplot as plt
from PySide2 import QtCore

from qudi.util.datastorage import TextDataStorage
from qudi.util.units import ScaledFloat

from .data_structures import ScanMode


class DataSavingMixin:
    """
    Mixin class providing data saving functionality for MotorScanLogic.
    
    This mixin provides:
    - Saving scan data to files (multiple formats based on scan mode)
    - Drawing figures/thumbnails for visual preview
    - Saving raw ODMR data per pixel
    - Saving position data with statistics
    
    The parent class must provide:
    - _scan_data: MotorScanData instance
    - _scan_state: Current ScanState
    - _thread_lock: RecursiveMutex for thread safety
    - _save_thumbnails: ConfigOption bool
    - _current_scan_folder: str path
    - _tracking_zero_crossing: float
    - _tracking_zero_crossing_history: list
    - module_default_data_dir: str property
    - module_state: Module state accessor
    - sigSaveStateChanged: Signal(bool)
    - log: Logger instance
    """

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
        from .data_structures import ScanState
        
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
