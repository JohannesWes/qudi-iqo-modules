# -*- coding: utf-8 -*-
"""
ODMR Frequency Tracking Logic Module

This module orchestrates ODMR resonance tracking by combining:
1. ODMR scanning (via OdmrLogic)
2. Linear fitting to extract error signal slope
3. PyRPL frequency lock control
4. Error signal streaming and monitoring

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level
directory of this distribution and on <https://github.com/Ulm-IQO/qudi-core/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
qudi. If not, see <https://www.gnu.org/licenses/>.
"""

__all__ = ['OdmrFrequencyTrackingLogic']

import numpy as np
from datetime import datetime
from typing import Tuple, Optional, Dict

from PySide2 import QtCore
from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.logic.odmr_logic import OdmrLogic
from qudi.util.mutex import RecursiveMutex


class OdmrFrequencyTrackingLogic(OdmrLogic):
    """
    Extended ODMR logic with frequency tracking capabilities.
    
    Inherits all ODMR scanning functionality and adds:
    - Linear fitting to extract error signal slope
    - PyRPL frequency lock control
    - Error signal streaming and monitoring

    This module coordinates ODMR scanning, resonance fitting, and closed-loop
    frequency tracking using PyRPL's hardware frequency lock.

    Example config:

        # Time series reader logic (handles streaming via TSR pattern)
        time_series_reader_logic:
            module.Class: 'time_series_reader_logic.TimeSeriesReaderLogic'
            options:
                max_frame_rate: 20
                channel_buffer_size: 100000
            connect:
                streamer: 'redpitaya_stream'

        # ODMR frequency tracking logic
        odmr_frequency_tracking_logic:
            module.Class: 'odmr_frequency_tracking_logic.OdmrFrequencyTrackingLogic'
            options:
                default_lock_bandwidth: 300  # Hz
                status_poll_interval: 0.5    # seconds (lock status only)
            connect:
                microwave: 'mw_source_synthnv'
                data_scanner: 'redpitaya_finite_sampling'
                odmr_lock_hw: 'redpitaya_odmr_lock'
                time_series_logic: 'time_series_reader_logic'
    """

    # =========================================================================
    # Additional Signals (tracking-specific)
    # =========================================================================

    # Fitting signals
    sigFitCompleted = QtCore.Signal(dict)  # {slope, offset, freq_min, freq_max, fit_data}
    sigFitFailed = QtCore.Signal(str)      # error message

    # Tracking signals
    sigLockStateChanged = QtCore.Signal(bool)  # locked
    sigStreamStateChanged = QtCore.Signal(bool)  # stream_active
    sigStreamModeChanged = QtCore.Signal(str)  # 'error' or 'correction'
    sigLockStatusUpdated = QtCore.Signal(dict)  # {locked, saturated, error_lsb, correction_hz}
    sigErrorDataUpdated = QtCore.Signal(object, object)  # times, error_data

    # =========================================================================
    # Additional Connectors (tracking-specific)
    # =========================================================================

    _odmr_lock_hw = Connector(name='odmr_lock_hw', interface='OdmrFreqLockInterface')
    _time_series_logic = Connector(name='time_series_logic', interface='TimeSeriesReaderLogic')

    # =========================================================================
    # Additional Config Options (tracking-specific)
    # =========================================================================

    _default_lock_bandwidth = ConfigOption('default_lock_bandwidth', default=300, missing='info')
    _status_poll_interval = ConfigOption('status_poll_interval', default=0.5, missing='info')  # seconds (lock status only)

    # =========================================================================
    # Additional Status Variables (tracking-specific)
    # =========================================================================

    _lock_bandwidth = StatusVar('lock_bandwidth', default=300)  # Hz
    _fit_range = StatusVar('fit_range', default=(2.86e9, 2.88e9))  # (freq_min, freq_max)
    _stream_mode = StatusVar('stream_mode', default='error')  # 'error' or 'correction'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = RecursiveMutex()

        # Tracking state (independent subsystems)
        self._lock_enabled = False
        self._stream_active = False
        self._status_polling_active = False
        self._last_fit_result = None  # {slope, offset, ...}

        # Status polling timer (for lock status only - streaming handled by TSR)
        self._status_timer = QtCore.QTimer()
        self._status_timer.timeout.connect(self._poll_lock_status)

    # =========================================================================
    # ODMR Scan Preparation (ensure proper hardware state before scanning)
    # =========================================================================

    def _prepare_for_odmr_scan(self):
        """
        Prepare hardware state for ODMR scanning.

        ODMR scans require:
        1. Lock disabled - so DDS output is not being corrected during measurement
        2. Input set to DEMOD - to measure the raw demodulated lock-in signal,
           not the FTW frequency correction

        This method updates the internal state and emits signals to synchronize
        the GUI with the actual hardware state. The GUI will reflect these changes
        in the streaming/lock control panels.
        """
        changes_made = False

        # Stop streaming if active (ODMR scan and streaming share hardware)
        if self._stream_active:
            self.log.info('Stopping error stream for ODMR scan')
            try:
                ts_logic = self._time_series_logic()
                if ts_logic.module_state() == 'locked':
                    ts_logic.stop_reading()
            except Exception as e:
                self.log.warning(f'Error stopping stream: {e}')
            self._stream_active = False
            self.sigStreamStateChanged.emit(False)
            changes_made = True

        # Disable lock if enabled (prevents frequency corrections during scan)
        if self._lock_enabled:
            self.log.info('Disabling frequency lock for ODMR scan')
            try:
                lock_hw = self._odmr_lock_hw()
                lock_hw.enable_lock(False)
            except Exception as e:
                self.log.error(f'Failed to disable lock: {e}')
            self._lock_enabled = False
            self.sigLockStateChanged.emit(False)
            changes_made = True

        # Ensure input is set to DEMOD (required for ODMR scan data)
        # This is critical: the scan module's input_select is shared between
        # scan mode and stream mode. If streaming was using FTW_CORR, we must
        # reset to DEMOD for proper ODMR measurements.
        if self._stream_mode != 'error':
            self.log.info('Setting stream mode to "error" (DEMOD input) for ODMR scan')
            self._stream_mode = 'error'
            self.sigStreamModeChanged.emit('error')
            changes_made = True

        # Always apply DEMOD input to hardware regardless of stream_mode change
        # This ensures the hardware register is set correctly even if mode was
        # already 'error' but hardware was in different state
        self._apply_stream_mode_to_hardware()

        if changes_made:
            self.log.info('Hardware state updated for ODMR scan (lock disabled, input=DEMOD)')
        else:
            self.log.debug('Hardware already in correct state for ODMR scan')

    @QtCore.Slot()
    def start_odmr_scan(self):
        """
        Override parent to prepare hardware before ODMR scan.

        Before starting the scan, this method ensures:
        1. Frequency lock is disabled (no corrections applied to DDS)
        2. Streaming is stopped (shared hardware resource)
        3. Input is set to DEMOD (not FTW_CORR which is for streaming only)

        The GUI is automatically updated via signals to reflect these changes,
        so the user sees the actual hardware state in the control panels.
        """
        # Prepare hardware state before scan
        self._prepare_for_odmr_scan()

        # Call parent implementation to run the actual ODMR scan
        super().start_odmr_scan()

    def on_activate(self):
        """Initialize module and connect hardware"""
        # Call parent activation (connects microwave and data scanner)
        super().on_activate()

        # Connect to TimeSeriesReaderLogic signals for data updates
        ts_logic = self._time_series_logic()
        ts_logic.sigNewRawData.connect(self._on_tsr_raw_data, QtCore.Qt.QueuedConnection)
        ts_logic.sigDataChanged.connect(self._on_tsr_data_changed, QtCore.Qt.QueuedConnection)
        ts_logic.sigStatusChanged.connect(self._on_tsr_status_changed, QtCore.Qt.QueuedConnection)

        # Set initial stream input mode on hardware (via TSR's streamer)
        self._apply_stream_mode_to_hardware()

        # Apply saved lock bandwidth
        if self._lock_bandwidth != self._default_lock_bandwidth:
            self._lock_bandwidth = self._default_lock_bandwidth

        self.log.info('ODMR Frequency Tracking Logic activated')

    def on_deactivate(self):
        """Cleanup and disconnect"""
        # Stop lock if active
        if self._lock_enabled:
            self.stop_tracking()

        # Stop streaming if active
        if self._stream_active:
            self.stop_error_stream()

        # Stop status polling
        self._status_timer.stop()

        # Disconnect from TSR signals
        try:
            ts_logic = self._time_series_logic()
            ts_logic.sigNewRawData.disconnect(self._on_tsr_raw_data)
            ts_logic.sigDataChanged.disconnect(self._on_tsr_data_changed)
            ts_logic.sigStatusChanged.disconnect(self._on_tsr_status_changed)
        except (RuntimeError, TypeError):
            # Signals may already be disconnected or TSR may be deactivated
            pass

        # Call parent deactivation
        super().on_deactivate()

        self.log.info('ODMR Frequency Tracking Logic deactivated')

    # =========================================================================
    # Fit stored ODMR scan data
    # =========================================================================

    def fit_resonance(self, freq_min: float, freq_max: float) -> Dict:
        """
        Fit linear function to ODMR data within specified range.

        Extracts the error signal slope (LSB/Hz) needed for lock configuration.
        Uses scan data from parent OdmrLogic class.

        Args:
            freq_min: Lower frequency bound [Hz]
            freq_max: Upper frequency bound [Hz]

        Returns:
            dict with keys: slope, offset, freq_min, freq_max, fit_data, r_squared

        Raises:
            ValueError: If no scan data available or invalid range
        """
        # Get scan data from parent class (OdmrLogic)
        try:
            # Use signal_data from parent - get first channel and first range
            channel_names = list(self.signal_data.keys())
            if not channel_names:
                raise ValueError('No ODMR scan data available. Run scan first.')
            
            freq = self.frequency_data[0]  # First frequency range
            signal = self.signal_data[channel_names[0]][0]  # First channel, first range
        except (AttributeError, IndexError, KeyError) as e:
            raise ValueError(f'No ODMR scan data available. Run scan first. Error: {e}')

        # Validate range
        if freq_min >= freq_max:
            raise ValueError(f'Invalid fit range: {freq_min} >= {freq_max}')

        if freq_min < freq.min() or freq_max > freq.max():
            raise ValueError(
                f'Fit range [{freq_min:.3e}, {freq_max:.3e}] outside scan range '
                f'[{freq.min():.3e}, {freq.max():.3e}]'
            )

        # Extract data within range
        mask = (freq >= freq_min) & (freq <= freq_max)
        freq_fit = freq[mask]
        signal_fit = signal[mask]

        if len(freq_fit) < 2:
            raise ValueError(f'Not enough points in fit range (need >= 2, got {len(freq_fit)})')

        # Linear fit: signal = slope * freq + offset
        coeffs = np.polyfit(freq_fit, signal_fit, deg=1)
        slope, offset = coeffs

        # Generate fit curve
        fit_data = slope * freq_fit + offset

        # Calculate R-squared
        ss_res = np.sum((signal_fit - fit_data) ** 2)
        ss_tot = np.sum((signal_fit - signal_fit.mean()) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0

        # Store result
        self._last_fit_result = {
            'slope': slope,
            'offset': offset,
            'freq_min': freq_min,
            'freq_max': freq_max,
            'fit_frequency': freq_fit,
            'fit_data': fit_data,
            'r_squared': r_squared,
            'timestamp': datetime.now()
        }

        # Save fit range
        self._fit_range = (freq_min, freq_max)

        # Emit success signal
        self.sigFitCompleted.emit(self._last_fit_result.copy())

        self.log.info(
            f'Resonance fit completed: slope={slope:.3e} LSB/Hz, '
            f'R²={r_squared:.4f}, range=[{freq_min/1e9:.4f}, {freq_max/1e9:.4f}] GHz'
        )

        return self._last_fit_result.copy()

    # =========================================================================
    # Frequency Lock Control
    # =========================================================================

    def configure_lock(self, slope_lsb_per_hz: Optional[float] = None,
                       bandwidth_hz: Optional[float] = None):
        """
        Configure PyRPL frequency lock with fitted parameters.

        Args:
            slope_lsb_per_hz: Discriminator slope from fit (if None, uses last fit)
            bandwidth_hz: Lock bandwidth (if None, uses saved value)
        """
        # Get slope from fit if not provided
        if slope_lsb_per_hz is None:
            if self._last_fit_result is None:
                raise ValueError('No fit result available. Run fit_resonance() first.')
            slope_lsb_per_hz = abs(self._last_fit_result['slope'])

        # Use saved bandwidth if not provided
        if bandwidth_hz is None:
            bandwidth_hz = self._lock_bandwidth
        else:
            self._lock_bandwidth = bandwidth_hz

        # Configure hardware lock
        # Thread-safe: MonitorClient uses RLock to serialize TCP socket access
        lock_hw = self._odmr_lock_hw()
        lock_hw.set_bandwidth(bandwidth_hz, slope_lsb_per_hz)

        self.log.info(
            f'Lock configured: bandwidth={bandwidth_hz} Hz, '
            f'slope={slope_lsb_per_hz:.3e} LSB/Hz'
        )

    def configure_lock_pi(self, bandwidth_hz: Optional[float] = None,
                          slope_lsb_per_hz: Optional[float] = None,
                          zero_ratio: float = 3.0):
        """
        Configure PyRPL frequency lock with PI control.

        Args:
            bandwidth_hz: Lock bandwidth (if None, uses saved value)
            slope_lsb_per_hz: Discriminator slope from fit (if None, uses last fit)
            zero_ratio: PI zero placement ratio α (default: 3.0)
                       Zero frequency = bandwidth_hz / zero_ratio
                       Range: [2.0, 4.0]
                       - α = 2.0: Aggressive (faster, may overshoot)
                       - α = 3.0: Balanced (recommended)
                       - α = 4.0: Conservative (slower, very stable)
        """
        # Get slope from fit if not provided
        if slope_lsb_per_hz is None:
            if self._last_fit_result is None:
                raise ValueError('No fit result available. Run fit_resonance() first.')
            slope_lsb_per_hz = abs(self._last_fit_result['slope'])

        # Use saved bandwidth if not provided
        if bandwidth_hz is None:
            bandwidth_hz = self._lock_bandwidth
        else:
            self._lock_bandwidth = bandwidth_hz

        # Configure hardware lock (PI mode)
        # Thread-safe: MonitorClient uses RLock to serialize TCP socket access
        lock_hw = self._odmr_lock_hw()
        lock_hw.set_bandwidth_pi(bandwidth_hz, slope_lsb_per_hz, zero_ratio)

        self.log.info(
            f'Lock configured (PI): bandwidth={bandwidth_hz} Hz, '
            f'slope={slope_lsb_per_hz:.3e} LSB/Hz, α={zero_ratio:.2f} '
            f'(zero at {bandwidth_hz/zero_ratio:.1f} Hz)'
        )

    def set_max_correction_hz(self, max_correction_hz: float) -> None:
        """
        Set maximum frequency correction (FTW saturation limit).

        The lock integrator output is clamped to ±max_correction_hz.
        When the correction hits this limit, the 'saturated' status flag is set.

        Args:
            max_correction_hz: Maximum correction magnitude in Hz (default: 1e6)
                              Typical range: 100 kHz to 10 MHz depending on
                              expected drift and tuning range.
        """
        lock_hw = self._odmr_lock_hw()
        lock_hw.set_max_correction_hz(max_correction_hz)
        self.log.info(f'Lock max correction set to {max_correction_hz/1e6:.3f} MHz')

    def get_max_correction_hz(self) -> float:
        """
        Get current maximum frequency correction setting.

        Returns:
            float: Maximum correction magnitude in Hz
        """
        lock_hw = self._odmr_lock_hw()
        return lock_hw.get_max_correction_hz()

    # =========================================================================
    # Error Stream Control (Independent of Lock)
    # =========================================================================

    def _apply_stream_mode_to_hardware(self):
        """
        Apply current stream mode to hardware via TSR's underlying streamer.

        Can be called while streaming is active - the FPGA will switch to the
        new input source seamlessly. Thread-safe via MonitorClient RLock.
        """
        ts_logic = self._time_series_logic()
        streamer = ts_logic._streamer()

        input_mode = 'demod' if self._stream_mode == 'error' else 'ftw_corr'
        streamer.set_stream_input(input_mode)

        self.log.debug(f'Hardware stream input set to: {input_mode}')

    def set_stream_mode(self, mode: str):
        """
        Set stream mode: 'error' or 'correction'.

        Can be called while streaming is active - the FPGA will switch to the
        new input source seamlessly. There may be a brief transition in the
        displayed data during the switch.

        Args:
            mode: 'error' for demod error signal, 'correction' for FTW frequency correction

        Raises:
            ValueError: If invalid mode
        """
        if mode not in ['error', 'correction']:
            raise ValueError(f'Invalid stream mode: {mode}. Must be "error" or "correction"')

        # Store mode
        self._stream_mode = mode

        # Apply to hardware via TSR's streamer (works during active streaming)
        self._apply_stream_mode_to_hardware()

        self.sigStreamModeChanged.emit(mode)

        input_mode = 'demod' if mode == 'error' else 'ftw_corr'
        self.log.info(f'Stream mode set to: {mode} (hardware input: {input_mode})')

    def start_error_stream(self):
        """
        Start error/correction signal streaming WITHOUT enabling frequency lock.

        Stream content depends on stream_mode setting:
        - 'error': Demodulated error signal (LSB)
        - 'correction': FTW frequency correction (Hz)

        Allows monitoring for diagnostics and verification
        before or during lock operation.

        Uses TimeSeriesReaderLogic for thread-safe, buffered data acquisition.
        """
        if self._stream_active:
            self.log.warning('Stream already active')
            return

        try:
            ts_logic = self._time_series_logic()

            # Ensure TSR is stopped before changing hardware input
            if ts_logic.module_state() == 'locked':
                ts_logic.stop_reading()

            # Ensure hardware input matches current mode
            self._apply_stream_mode_to_hardware()

            # Start TSR streaming (handles buffering, thread safety, etc.)
            ts_logic.start_reading()

            # Mark stream as active
            self._stream_active = True
            self.sigStreamStateChanged.emit(True)

            # Start status polling timer if not already running (for lock status only)
            if not self._status_polling_active:
                self._status_timer.start(int(self._status_poll_interval * 1000))
                self._status_polling_active = True

            self.log.info(f'Error signal streaming started (mode: {self._stream_mode})')

        except Exception as e:
            self.log.error(f'Failed to start error stream: {e}')
            self._stream_active = False
            self.sigStreamStateChanged.emit(False)
            raise

    def stop_error_stream(self):
        """
        Stop error signal streaming WITHOUT disabling frequency lock.

        Stops data acquisition but leaves lock running if enabled.
        """
        if not self._stream_active:
            return

        try:
            # Stop TSR streaming
            ts_logic = self._time_series_logic()
            if ts_logic.module_state() == 'locked':
                ts_logic.stop_reading()

            # Mark stream as inactive
            self._stream_active = False
            self.sigStreamStateChanged.emit(False)

            # Stop status polling timer only if lock is also inactive
            if not self._lock_enabled and self._status_polling_active:
                self._status_timer.stop()
                self._status_polling_active = False

            self.log.info('Error signal streaming stopped')

        except Exception as e:
            self.log.error(f'Error stopping stream: {e}')

    # =========================================================================
    # Frequency Lock Control
    # =========================================================================

    def start_tracking(self):
        """
        Start frequency tracking (enable lock).

        Automatically starts error signal streaming if not already active.
        Lock requires active error signal to function.
        """
        if self._lock_enabled:
            self.log.warning('Lock already enabled')
            return

        try:
            # Enable hardware lock (single register write to FPGA)
            # Thread-safe: MonitorClient uses RLock to serialize TCP socket access
            lock_hw = self._odmr_lock_hw()
            lock_hw.enable_lock(True)

            # Mark lock as enabled
            self._lock_enabled = True

            # Start streaming if not already active
            if not self._stream_active:
                ts_logic = self._time_series_logic()
                if ts_logic.module_state() != 'locked':
                    ts_logic.start_reading()
                self._stream_active = True
                self.sigStreamStateChanged.emit(True)

            self.sigLockStateChanged.emit(True)

            # Ensure status polling is active
            if not self._status_polling_active:
                self._status_timer.start(int(self._status_poll_interval * 1000))
                self._status_polling_active = True

            self.log.info('Frequency lock enabled')

        except Exception as e:
            self.log.error(f'Failed to enable lock: {e}', exc_info=True)
            self._lock_enabled = False
            self.sigLockStateChanged.emit(False)
            raise

    def stop_tracking(self):
        """
        Stop frequency tracking (disable lock).

        Disables lock but leaves error signal streaming active.
        Call stop_error_stream() separately if streaming should also stop.
        """
        if not self._lock_enabled:
            return

        try:
            # Disable hardware lock
            # Thread-safe: MonitorClient uses RLock to serialize TCP socket access
            lock_hw = self._odmr_lock_hw()
            lock_hw.enable_lock(False)

            # Mark lock as disabled
            self._lock_enabled = False

            self.sigLockStateChanged.emit(False)

            # Stop status polling timer only if stream is also inactive
            if not self._stream_active and self._status_polling_active:
                self._status_timer.stop()
                self._status_polling_active = False

            self.log.info('Frequency lock disabled')

        except Exception as e:
            self.log.error(f'Error disabling lock: {e}', exc_info=True)

    def clear_integrator(self):
        """
        Clear lock integrator state without disabling lock.

        Useful for re-acquiring lock after disturbances or when error
        signal has drifted far from zero.
        """
        # Clear integrator
        # Thread-safe: MonitorClient uses RLock to serialize TCP socket access
        lock_hw = self._odmr_lock_hw()
        lock_hw.clear()
        self.log.info('Lock integrator cleared')

    # =========================================================================
    # Lock Status Polling (independent of data streaming)
    # =========================================================================

    @QtCore.Slot()
    def _poll_lock_status(self):
        """
        Poll lock status only (called by timer).

        Data streaming is handled by TimeSeriesReaderLogic signals.
        This timer only monitors the lock hardware status for GUI display.

        Thread-safe: MonitorClient uses RLock to serialize TCP socket access,
        so polling can occur concurrently with streaming without conflicts.
        """
        if not self._lock_enabled:
            return

        try:
            lock_hw = self._odmr_lock_hw()
            status = lock_hw.get_status()
            self.sigLockStatusUpdated.emit(status)
        except Exception as e:
            # Catch all exceptions to prevent timer from stopping
            self.log.debug(f'Lock status poll failed: {e}')

    # =========================================================================
    # TSR Signal Handlers (data streaming via TimeSeriesReaderLogic)
    # =========================================================================

    @QtCore.Slot(object, object)
    def _on_tsr_raw_data(self, data, times):
        """
        Handle raw data from TimeSeriesReaderLogic.

        Forwards data to tracking GUI via sigErrorDataUpdated.
        This replaces the old polling-based data acquisition.

        Args:
            data: Raw samples from streamer (1D numpy array)
            times: Optional timestamp array (may be None for constant sample rate)
        """
        if not self._stream_active:
            return

        # Get time axis from TSR if not provided
        if times is None:
            ts_logic = self._time_series_logic()
            trace_times, _ = ts_logic.trace_data
            times = trace_times

        # Forward to GUI
        # Note: TSR already handles buffering, so we emit the trace data
        ts_logic = self._time_series_logic()
        trace_times, trace_data = ts_logic.trace_data

        if trace_data:
            # Get first channel data
            channel_name = list(trace_data.keys())[0]
            self.sigErrorDataUpdated.emit(trace_times, trace_data[channel_name])

    @QtCore.Slot(object, object, object, object)
    def _on_tsr_data_changed(self, trace_times, trace_data, avg_times, avg_data):
        """
        Handle processed trace data from TimeSeriesReaderLogic.

        This signal provides the full trace window with moving average applied.
        Can be used for GUI display if processed data is preferred.

        Args:
            trace_times: Time axis array
            trace_data: Dictionary of {channel_name: data_array}
            avg_times: Time axis for averaged data
            avg_data: Dictionary of averaged data (may be None)
        """
        if not self._stream_active:
            return

        # Forward processed data to GUI
        if trace_data:
            channel_name = list(trace_data.keys())[0]
            self.sigErrorDataUpdated.emit(trace_times, trace_data[channel_name])

    @QtCore.Slot(bool, bool)
    def _on_tsr_status_changed(self, is_running, is_recording):
        """
        Handle status changes from TimeSeriesReaderLogic.

        Synchronizes our stream_active state with TSR's actual state.

        Args:
            is_running: True if TSR is actively streaming
            is_recording: True if TSR is recording to file
        """
        # Update our stream state to match TSR
        if self._stream_active and not is_running:
            # TSR stopped unexpectedly
            self.log.warning('TimeSeriesReaderLogic stopped unexpectedly')
            self._stream_active = False
            self.sigStreamStateChanged.emit(False)

    # =========================================================================
    # Properties
    # =========================================================================

    @property
    def lock_enabled(self) -> bool:
        """True if frequency lock is enabled"""
        return self._lock_enabled

    @property
    def stream_active(self) -> bool:
        """True if error signal streaming is active"""
        return self._stream_active

    @property
    def stream_mode(self) -> str:
        """Current stream mode: 'error' or 'correction'"""
        return self._stream_mode

    @property
    def tracking_active(self) -> bool:
        """True if frequency tracking is active (DEPRECATED: use lock_enabled)"""
        return self._lock_enabled

    @property
    def last_scan_data(self) -> Optional[Dict]:
        """Most recent ODMR scan data (from parent OdmrLogic)"""
        # Access scan data from parent OdmrLogic class
        try:
            if self.frequency_data is not None and self.signal_data:
                return {
                    'frequency': self.frequency_data,
                    'signal': self.signal_data
                }
        except (AttributeError, KeyError):
            pass
        return None

    @property
    def last_fit_result(self) -> Optional[Dict]:
        """Most recent fit result"""
        return self._last_fit_result.copy() if self._last_fit_result else None

    @property
    def lock_bandwidth(self) -> float:
        """Current lock bandwidth [Hz]"""
        return self._lock_bandwidth

    @lock_bandwidth.setter
    def lock_bandwidth(self, value: float):
        """Set lock bandwidth [Hz]"""
        if value <= 0:
            raise ValueError('Bandwidth must be positive')
        self._lock_bandwidth = value
