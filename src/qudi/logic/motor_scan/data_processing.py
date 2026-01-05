# -*- coding: utf-8 -*-
"""
Data processing mixin for MotorScanLogic.

Handles data acquisition for different scan modes: ODMR, streaming, frequency tracking.
"""

import os
import numpy as np
from typing import Dict

from PySide2 import QtCore

from .data_structures import ScanMode

# FTW (Frequency Tuning Word) conversion constant for Red Pitaya @ 125 MHz
# FTW = frequency_hz * FTW_PER_HZ; frequency_hz = ftw / FTW_PER_HZ
FTW_PER_HZ = (2**32) / 125e6  # ≈ 34.359738


class DataProcessingMixin:
    """
    Mixin class providing data acquisition and processing for MotorScanLogic.
    
    This mixin provides:
    - Time series data handling (CONTINUOUS_STREAM mode)
    - Frequency tracking data processing (CONTINUOUS_FREQ_TRACK mode)
    - ODMR scan processing and fitting (STEP_ODMR mode)
    - Lock status monitoring
    
    The parent class must provide:
    - _time_series_logic: Connector to time series logic
    - _odmr_logic: Connector to ODMR logic
    - _odmr_frequency_tracking_logic: Connector to tracking logic
    - _scan_data: MotorScanData instance
    - _scan_state: Current ScanState
    - _active_scan_mode: Current scan mode value
    - _ts_raw_data_buffer: Dict for buffering raw data
    - _ts_connected: bool flag for signal connection
    - _tracking_zero_crossing: float, baseline frequency
    - _fit_function: callable for ODMR fitting
    - Various ConfigOptions for fitting parameters
    - log: Logger instance
    """

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
        from .data_structures import ScanState
        
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

    @QtCore.Slot()
    def _on_lock_status_poll_timeout(self):
        """
        Poll lock status during CONTINUOUS_FREQ_TRACK mode.

        Auto-pauses the scan if lock is lost, emitting sigLockLostDuringScan.
        Emits sigLockStatusUpdated for GUI indicator updates.
        """
        from .data_structures import ScanState
        
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
