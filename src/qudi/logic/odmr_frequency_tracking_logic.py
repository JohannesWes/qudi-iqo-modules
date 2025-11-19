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

        odmr_frequency_tracking_logic:
            module.Class: 'odmr_frequency_tracking_logic.OdmrFrequencyTrackingLogic'
            options:
                default_lock_bandwidth: 300  # Hz
                error_buffer_size: 10000     # samples
            connect:
                microwave: 'mw_source_synthnv'
                data_scanner: 'redpitaya_finite_sampling'
                odmr_lock_hw: 'redpitaya_odmr_lock'
                error_streamer: 'redpitaya_stream'
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
    _error_streamer = Connector(name='error_streamer', interface='DataInStreamInterface')

    # =========================================================================
    # Additional Config Options (tracking-specific)
    # =========================================================================

    _default_lock_bandwidth = ConfigOption('default_lock_bandwidth', default=300, missing='info')
    _error_buffer_size = ConfigOption('error_buffer_size', default=10000, missing='info')
    _status_poll_interval = ConfigOption('status_poll_interval', default=0.1, missing='info')  # seconds

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

        # Error stream management
        self._error_buffer = None  # Circular buffer for error data
        self._error_times = None   # Time axis for error data
        self._error_write_pos = 0

        # Status polling timer
        self._status_timer = QtCore.QTimer()
        self._status_timer.timeout.connect(self._poll_lock_status)

    def on_activate(self):
        """Initialize module and connect hardware"""
        # Call parent activation (connects microwave and data scanner)
        super().on_activate()
        
        # Initialize error buffer
        self._error_buffer = np.zeros(self._error_buffer_size, dtype=np.float64)
        self._error_times = np.zeros(self._error_buffer_size, dtype=np.float64)
        self._error_write_pos = 0

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
        lock_hw = self._odmr_lock_hw()
        lock_hw.set_bandwidth_pi(bandwidth_hz, slope_lsb_per_hz, zero_ratio)

        self.log.info(
            f'Lock configured (PI): bandwidth={bandwidth_hz} Hz, '
            f'slope={slope_lsb_per_hz:.3e} LSB/Hz, α={zero_ratio:.2f} '
            f'(zero at {bandwidth_hz/zero_ratio:.1f} Hz)'
        )

    # =========================================================================
    # Error Stream Control (Independent of Lock)
    # =========================================================================

    def set_stream_mode(self, mode: str):
        """
        Set stream mode: 'error' or 'correction'.

        Args:
            mode: 'error' for demod error signal, 'correction' for FTW frequency correction

        Raises:
            ValueError: If invalid mode
            RuntimeError: If stream is active
        """
        if mode not in ['error', 'correction']:
            raise ValueError(f'Invalid stream mode: {mode}. Must be "error" or "correction"')

        if self._stream_active:
            raise RuntimeError('Cannot change stream mode while streaming is active. Stop stream first.')

        # Update hardware input
        streamer = self._error_streamer()
        input_mode = 'demod' if mode == 'error' else 'ftw_corr'
        streamer.set_stream_input(input_mode)

        # Store mode
        self._stream_mode = mode
        self.sigStreamModeChanged.emit(mode)

        self.log.info(f'Stream mode set to: {mode} (hardware input: {input_mode})')

    def start_error_stream(self):
        """
        Start error/correction signal streaming WITHOUT enabling frequency lock.

        Stream content depends on stream_mode setting:
        - 'error': Demodulated error signal (LSB)
        - 'correction': FTW frequency correction (Hz)

        Allows monitoring for diagnostics and verification
        before or during lock operation.
        """
        if self._stream_active:
            self.log.warning('Stream already active')
            return

        try:
            # Ensure hardware input matches current mode
            streamer = self._error_streamer()
            input_mode = 'demod' if self._stream_mode == 'error' else 'ftw_corr'
            streamer.set_stream_input(input_mode)

            # Start streaming
            streamer.start_stream()

            # Mark stream as active
            self._stream_active = True
            self.sigStreamStateChanged.emit(True)

            # Start status polling timer if not already running
            if not self._status_polling_active:
                self._status_timer.start(int(self._status_poll_interval * 1000))
                self._status_polling_active = True

            self.log.info('Error signal streaming started')

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
            # Stop error signal streaming
            streamer = self._error_streamer()
            streamer.stop_stream()

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
            # Ensure error stream is running (required for lock operation)
            if not self._stream_active:
                self.log.warning('Error stream not active, starting automatically')
                self.start_error_stream()

            # Enable hardware lock
            lock_hw = self._odmr_lock_hw()
            lock_hw.enable_lock(True)

            # Mark lock as enabled
            self._lock_enabled = True
            self.sigLockStateChanged.emit(True)

            # Ensure status polling is active
            if not self._status_polling_active:
                self._status_timer.start(int(self._status_poll_interval * 1000))
                self._status_polling_active = True

            self.log.info('Frequency lock enabled')

        except Exception as e:
            self.log.error(f'Failed to enable lock: {e}')
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
            self.log.error(f'Error disabling lock: {e}')

    def clear_integrator(self):
        """
        Clear lock integrator state without disabling lock.

        Useful for re-acquiring lock after disturbances or when error
        signal has drifted far from zero.
        """
        lock_hw = self._odmr_lock_hw()
        lock_hw.clear()
        self.log.info('Lock integrator cleared')

    # =========================================================================
    # Error Stream Management
    # =========================================================================

    @QtCore.Slot()
    def _poll_lock_status(self):
        """
        Poll lock status and/or read error stream (called by timer).

        Handles independent operation of lock and stream subsystems.
        """
        # Read lock status if lock is enabled
        if self._lock_enabled:
            try:
                lock_hw = self._odmr_lock_hw()
                status = lock_hw.get_status()
                self.sigLockStatusUpdated.emit(status)
            except Exception as e:
                self.log.error(f'Error polling lock status: {e}')

        # Read error stream if streaming is active
        if self._stream_active:
            try:
                streamer = self._error_streamer()
                data, times = streamer.read_data()  # Non-blocking

                if data is not None and len(data) > 0:
                    self._add_to_error_buffer(data.ravel(), times)
            except Exception as e:
                self.log.error(f'Error reading error stream: {e}')

    def _add_to_error_buffer(self, data: np.ndarray, times: Optional[np.ndarray]):
        """Add new error data to circular buffer"""
        n_samples = len(data)
        if n_samples == 0:
            return

        # Generate times if not provided (constant sample rate)
        if times is None:
            streamer = self._error_streamer()
            dt = 1.0 / streamer.sample_rate
            if self._error_write_pos > 0:
                t_start = self._error_times[self._error_write_pos - 1] + dt
            else:
                t_start = 0.0
            times = t_start + np.arange(n_samples) * dt

        # Write to circular buffer
        buffer_size = self._error_buffer_size
        for i in range(n_samples):
            self._error_buffer[self._error_write_pos] = data[i]
            self._error_times[self._error_write_pos] = times[i]
            self._error_write_pos = (self._error_write_pos + 1) % buffer_size

        # Emit updated data (last N samples for display)
        self.sigErrorDataUpdated.emit(
            self._error_times.copy(),
            self._error_buffer.copy()
        )

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
        """Most recent ODMR scan data"""
        return self._last_scan_data.copy() if self._last_scan_data else None

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
