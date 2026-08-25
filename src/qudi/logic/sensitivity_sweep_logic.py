# -*- coding: utf-8 -*-
"""
Sensitivity Measurement Parameter Sweep Logic Module

This module orchestrates automated ODMR-based magnetic field sensitivity measurements
with systematic parameter sweeps over microwave power, FM modulation frequency,
and FM deviation.

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

__all__ = ['SensitivitySweepLogic']

import numpy as np
import pandas as pd
import time
import os
import json
from datetime import datetime
from itertools import product
from typing import Tuple, Dict, List, Optional, Any
from PySide2 import QtCore

from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from qudi.core.statusvariable import StatusVar
from qudi.core.module import LogicBase
from qudi.util.mutex import RecursiveMutex
from qudi.util.datastorage import TextDataStorage, ImageFormat

# Import analysis functions from existing code
# Add parent directories to path to find my_software
import sys
import os
# Get path to qudi-core root (3 levels up from this file)
qudi_core_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if qudi_core_root not in sys.path:
    sys.path.insert(0, qudi_core_root)

from my_software.tools.fitting import fit_hyperfine
from my_software.sensitivity_msmt.auswertung.sensitivity_auswertung_modular import (
    magnetic_field_from_voltages,
    plot_asds,
    plot_magnetic_field_time_traces
)

# Import visualization module for summary plots
from qudi.logic.sensitivity_sweep_visualizer import SensitivitySweepVisualizer


class SensitivitySweepLogic(LogicBase):
    """
    Logic module for automated ODMR-based magnetic field sensitivity parameter sweeps.

    This module orchestrates the full measurement workflow:
    1. Configure microwave source (power, FM frequency, FM deviation)
    2. Run ODMR scan and fit resonance to find zero-crossing
    3. Set CW microwave to zero-crossing frequency
    4. Record time series data for sensitivity measurement
    5. Calculate amplitude spectral density and magnetic field sensitivity
    6. Repeat for all parameter combinations

    Supports pausable/resumable operation and real-time progress reporting.

    Example config:

        sensitivity_sweep_logic:
            module.Class: 'sensitivity_sweep_logic.SensitivitySweepLogic'
            options:
                thermal_stabilization_time: 180
                sweep_loop_order: ['power', 'f_mod', 'f_dev']
                which_zero_crossing: 2
                n_most_prominent_peaks: 5
                min_fit_amplitude: 0.005
                min_feature_height: 0.003
                include_off_resonant_measurement: False
            connectors:
                odmr_logic: 'odmr_logic'
                time_series_logic: 'time_series_reader_logic'
    """

    # =========================================================================
    # Connectors
    # =========================================================================

    _odmr_logic = Connector(name='odmr_logic', interface='OdmrLogic')
    _time_series_logic = Connector(name='time_series_logic', interface='TimeSeriesReaderLogic')

    # =========================================================================
    # Config Options
    # =========================================================================

    _thermal_stabilization_time = ConfigOption(
        name='thermal_stabilization_time',
        default=180,
        missing='info'
    )

    _sweep_loop_order = ConfigOption(
        name='sweep_loop_order',
        default=['power', 'f_mod', 'f_dev'],
        missing='info'
    )

    _which_zero_crossing = ConfigOption(
        name='which_zero_crossing',
        default=2,
        missing='info'
    )

    _n_most_prominent_peaks = ConfigOption(
        name='n_most_prominent_peaks',
        default=5,
        missing='info'
    )

    _min_fit_amplitude = ConfigOption(
        name='min_fit_amplitude',
        default=0.005,
        missing='info'
    )

    _min_feature_height = ConfigOption(
        name='min_feature_height',
        default=0.003,
        missing='info'
    )

    _include_off_resonant_measurement = ConfigOption(
        name='include_off_resonant_measurement',
        default=False,
        missing='info'
    )

    _off_resonant_offset_hz = ConfigOption(
        name='off_resonant_offset_hz',
        default=30e6,
        missing='info'
    )

    _default_f_enbw = ConfigOption(
        name='default_f_enbw',
        default=500.0,
        missing='info'
    )  # Default Equivalent Noise Bandwidth of lock-in filter in Hz

    # =========================================================================
    # Status Variables (Persistent State)
    # =========================================================================

    _sweep_state = StatusVar(name='sweep_state', default='idle')  # idle, running, paused, cancelled
    _current_sweep_index = StatusVar(name='current_sweep_index', default=0)
    _sweep_parameters = StatusVar(name='sweep_parameters', default={})
    _odmr_parameters = StatusVar(name='odmr_parameters', default={})
    _stream_parameters = StatusVar(name='stream_parameters', default={})
    _results_list = StatusVar(name='results_list', default=[])
    _best_sensitivity = StatusVar(name='best_sensitivity', default=np.inf)
    _best_parameters = StatusVar(name='best_parameters', default={})
    _current_folder = StatusVar(name='current_folder', default='')

    # =========================================================================
    # Signals
    # =========================================================================

    sigSweepStarted = QtCore.Signal(int)  # total_points
    sigPointStarted = QtCore.Signal(int, dict)  # index, params
    sigPointCompleted = QtCore.Signal(int, dict)  # index, results
    sigSweepProgress = QtCore.Signal(dict)  # {current_idx, total, best_sens, best_params, ...}
    sigSweepPaused = QtCore.Signal()
    sigSweepResumed = QtCore.Signal()
    sigSweepFinished = QtCore.Signal(str)  # results_folder_path
    sigSweepCancelled = QtCore.Signal()
    sigError = QtCore.Signal(str)  # error_message

    # Real-time data for GUI plotting
    sigOdmrDataReady = QtCore.Signal(object, object)  # frequencies, signal
    sigFitDataReady = QtCore.Signal(dict)  # fit_result
    sigASDDataReady = QtCore.Signal(object, object, float)  # frequencies, asd_data, sensitivity
    sigTimeTraceReady = QtCore.Signal(object, object)  # times, b_field_data

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = RecursiveMutex()

        # Sweep execution control
        self._pause_requested = False
        self._cancel_requested = False

        # Current measurement data (temporary)
        self._parameter_combinations = []
        self._total_combinations = 0

    def on_activate(self):
        """Initialize the logic module."""
        try:
            # Verify connectors are valid
            odmr = self._odmr_logic()
            ts_logic = self._time_series_logic()

            self.log.info(f'Connected to ODMR logic: {odmr}')
            self.log.info(f'Connected to Time Series logic: {ts_logic}')

            # Initialize state
            if self._sweep_state not in ['idle', 'paused']:
                self._sweep_state = 'idle'

            self.log.info('Sensitivity Sweep Logic activated')

        except Exception as e:
            self.log.error(f'Failed to activate Sensitivity Sweep Logic: {e}')
            raise

    def on_deactivate(self):
        """Clean up resources."""
        try:
            # If sweep is running, stop it
            if self._sweep_state == 'running':
                self.cancel_sweep()
                time.sleep(0.5)  # Allow cleanup

            self.log.info('Sensitivity Sweep Logic deactivated')

        except Exception as e:
            self.log.error(f'Error during deactivation: {e}')

    # =========================================================================
    # Internal Methods - Event Processing
    # =========================================================================

    def _sleep_interruptible(self, seconds: float, check_interval: float = 0.5) -> bool:
        """
        Sleep for the specified duration while remaining responsive to pause/cancel.

        This method processes Qt events periodically during the sleep, allowing
        queued signals (like pause_sweep, cancel_sweep) to be processed.

        Args:
            seconds: Total sleep duration in seconds
            check_interval: How often to check for interrupts (default 0.5s)

        Returns:
            True if sleep completed normally, False if interrupted by pause/cancel
        """
        elapsed = 0.0
        while elapsed < seconds:
            # Process pending events (including pause/cancel signals)
            QtCore.QCoreApplication.processEvents()

            # Check if we should abort
            if self._pause_requested or self._cancel_requested:
                return False

            # Sleep for a short interval
            sleep_time = min(check_interval, seconds - elapsed)
            time.sleep(sleep_time)
            elapsed += sleep_time

        # Final event processing
        QtCore.QCoreApplication.processEvents()
        return not (self._pause_requested or self._cancel_requested)

    def _process_events_and_check_abort(self) -> bool:
        """
        Process pending Qt events and check if sweep should abort.

        Returns:
            True if sweep should continue, False if pause/cancel requested
        """
        QtCore.QCoreApplication.processEvents()
        return not (self._pause_requested or self._cancel_requested)

    # =========================================================================
    # Public Methods - Sweep Configuration
    # =========================================================================

    @QtCore.Slot(dict, dict, dict)
    def configure_sweep(self,
                       sweep_params: Dict[str, np.ndarray],
                       odmr_params: Dict[str, Any],
                       stream_params: Dict[str, Any]) -> None:
        """
        Configure parameter sweep settings.

        Args:
            sweep_params: Dictionary with keys 'power', 'f_mod', 'f_dev'
                         Each value is a numpy array of parameter values to sweep
                         Example: {'power': np.array([-25, -20, -15]),
                                  'f_mod': np.array([16e3, 20e3, 25e3]),
                                  'f_dev': np.array([500, 550, 600])}

            odmr_params: ODMR scan configuration
                        {'frequency_start': 2.86e9,
                         'frequency_stop': 2.88e9,
                         'frequency_points': 1001,
                         'run_time': 60,
                         'data_rate': 1000,
                         'multi_freq_mode': 'triple'}

            stream_params: Time series recording configuration
                          {'n_time_traces': 32,
                           'trace_duration': 1.0,  # seconds per trace
                           'data_rate': 1000}
        """
        with self._thread_lock:
            if self._sweep_state == 'running':
                raise RuntimeError('Cannot configure sweep while it is running')

            # Validate sweep parameters
            required_keys = ['power', 'f_mod', 'f_dev']
            for key in required_keys:
                if key not in sweep_params:
                    raise ValueError(f'Missing required sweep parameter: {key}')
                if not isinstance(sweep_params[key], (np.ndarray, list)):
                    raise ValueError(f'Sweep parameter {key} must be array-like')

            # Store configuration
            self._sweep_parameters = {
                'power': np.array(sweep_params['power']),
                'f_mod': np.array(sweep_params['f_mod']),
                'f_dev': np.array(sweep_params['f_dev'])
            }
            self._odmr_parameters = odmr_params.copy()
            self._stream_parameters = stream_params.copy()

            # Generate parameter combinations
            self._generate_parameter_combinations()

            self.log.info(
                f'Sweep configured: {self._total_combinations} total measurements'
            )
            self.log.info(
                f'Loop order: {self._sweep_loop_order}'
            )

    def _generate_parameter_combinations(self):
        """Generate all parameter combinations in the specified loop order."""
        param_dict = {
            'power': self._sweep_parameters['power'],
            'f_mod': self._sweep_parameters['f_mod'],
            'f_dev': self._sweep_parameters['f_dev']
        }

        # Validate loop order
        for param in self._sweep_loop_order:
            if param not in param_dict:
                raise ValueError(
                    f'Invalid loop order parameter: {param}. '
                    f'Must be one of {list(param_dict.keys())}'
                )

        # Create combinations in specified order
        ordered_arrays = [param_dict[p] for p in self._sweep_loop_order]
        self._parameter_combinations = list(product(*ordered_arrays))
        self._total_combinations = len(self._parameter_combinations)

        self.log.debug(f'Generated {self._total_combinations} parameter combinations')

    # =========================================================================
    # Public Methods - Sweep Control
    # =========================================================================

    @QtCore.Slot(dict, dict, dict)
    def configure_and_start_sweep(self, sweep_params: Dict[str, Any],
                                   odmr_params: Dict[str, Any],
                                   stream_params: Dict[str, Any]) -> None:
        """
        Configure and start sweep - single slot for GUI signal connection.

        This method ensures both configure and start happen on the logic thread,
        which is critical for proper pause/cancel functionality.

        Args:
            sweep_params: Parameter arrays for sweep
            odmr_params: ODMR scan configuration
            stream_params: Time series recording configuration
        """
        try:
            self.configure_sweep(sweep_params, odmr_params, stream_params)
            self.start_sweep()
        except Exception as e:
            self.log.error(f'Failed to start sweep: {e}', exc_info=True)
            self.sigError.emit(f'Failed to start sweep: {str(e)}')

    @QtCore.Slot()
    def start_sweep(self) -> None:
        """Start the parameter sweep from the beginning."""
        with self._thread_lock:
            if self._sweep_state == 'running':
                self.log.warning('Sweep already running')
                return

            if not self._parameter_combinations:
                raise RuntimeError('No sweep configured. Call configure_sweep() first.')

            # Reset state
            self._sweep_state = 'running'
            self._current_sweep_index = 0
            self._results_list = []
            self._best_sensitivity = np.inf
            self._best_parameters = {}
            self._pause_requested = False
            self._cancel_requested = False

            # Create data folder
            self._create_measurement_folder()

            # Emit start signal
            self.sigSweepStarted.emit(self._total_combinations)

            self.log.info(f'Starting sweep: {self._total_combinations} measurements')

            # Lock module and start sweep loop in thread
            self.module_state.lock()
            QtCore.QTimer.singleShot(0, self._run_sweep_loop)

    @QtCore.Slot()
    def pause_sweep(self) -> None:
        """Pause the running sweep (can be resumed later)."""
        with self._thread_lock:
            if self._sweep_state != 'running':
                self.log.warning('No sweep running to pause')
                return

            self._pause_requested = True
            self.log.info('Pause requested - will pause after current measurement')

    @QtCore.Slot()
    def resume_sweep(self) -> None:
        """Resume a paused sweep."""
        with self._thread_lock:
            if self._sweep_state != 'paused':
                self.log.warning('No paused sweep to resume')
                return

            self._sweep_state = 'running'
            self._pause_requested = False

            self.sigSweepResumed.emit()
            self.log.info(f'Resuming sweep from point {self._current_sweep_index + 1}')

            # Continue sweep loop
            self.module_state.lock()
            QtCore.QTimer.singleShot(0, self._run_sweep_loop)

    @QtCore.Slot()
    def cancel_sweep(self) -> None:
        """Cancel the running or paused sweep."""
        with self._thread_lock:
            if self._sweep_state not in ['running', 'paused']:
                self.log.warning('No sweep to cancel')
                return

            self._cancel_requested = True
            self.log.info('Cancel requested - will stop after current measurement')

    def get_sweep_status(self) -> Dict[str, Any]:
        """
        Get current sweep status.

        Returns:
            Dictionary with status information:
            {
                'state': 'idle'/'running'/'paused'/'cancelled',
                'current_index': int,
                'total_points': int,
                'best_sensitivity': float,
                'best_parameters': dict,
                'current_folder': str
            }
        """
        with self._thread_lock:
            return {
                'state': self._sweep_state,
                'current_index': self._current_sweep_index,
                'total_points': self._total_combinations,
                'best_sensitivity': self._best_sensitivity,
                'best_parameters': self._best_parameters.copy(),
                'current_folder': self._current_folder
            }

    def get_current_results(self) -> pd.DataFrame:
        """
        Get current results as pandas DataFrame.

        Returns:
            DataFrame with columns for parameters and sensitivity results
        """
        with self._thread_lock:
            if not self._results_list:
                return pd.DataFrame()
            return pd.DataFrame(self._results_list)

    # =========================================================================
    # Internal Methods - Sweep Loop
    # =========================================================================

    @QtCore.Slot()
    def _run_sweep_loop(self):
        """Main sweep loop - runs in logic thread."""
        try:
            # Process measurements from current_index to end
            while self._current_sweep_index < self._total_combinations:
                # CRITICAL: Process pending Qt events to allow pause/cancel signals through
                # Without this, queued signals cannot be processed while the loop is running
                QtCore.QCoreApplication.processEvents()

                # Check for pause/cancel requests
                if self._pause_requested:
                    self._handle_pause()
                    return

                if self._cancel_requested:
                    self._handle_cancel()
                    return

                # Get current parameter combination
                idx = self._current_sweep_index
                combination = self._parameter_combinations[idx]
                param_values = dict(zip(self._sweep_loop_order, combination))

                # Emit point started signal
                self.sigPointStarted.emit(idx, param_values)

                # Perform measurement
                try:
                    result = self._measure_single_point(idx, param_values)
                    self._results_list.append(result)

                    # Update best result
                    if not np.isnan(result.get('sensitivity_nT_rtHz', np.nan)):
                        if result['sensitivity_nT_rtHz'] < self._best_sensitivity:
                            self._best_sensitivity = result['sensitivity_nT_rtHz']
                            self._best_parameters = param_values.copy()
                            self.log.info(
                                f'New best sensitivity: {self._best_sensitivity:.3f} nT/sqrtHz'
                            )

                    # Emit point completed signal
                    self.sigPointCompleted.emit(idx, result)

                except InterruptedError:
                    # Sweep was interrupted by pause/cancel - don't treat as error
                    # The pause/cancel will be handled at the top of the next loop iteration
                    self.log.debug('Measurement interrupted, checking pause/cancel state')
                    continue

                except Exception as e:
                    self.log.error(f'Error measuring point {idx + 1}: {e}', exc_info=True)
                    # Add failed result
                    result = {
                        'power_dbm': param_values['power'],
                        'f_mod_hz': param_values['f_mod'],
                        'f_dev_khz': param_values['f_dev'],
                        'sensitivity_nT_rtHz': np.nan,
                        'error': str(e)
                    }
                    self._results_list.append(result)
                    self.sigError.emit(f'Point {idx + 1} failed: {str(e)}')

                # Emit progress update
                progress = {
                    'current_idx': idx + 1,
                    'total': self._total_combinations,
                    'best_sens': self._best_sensitivity,
                    'best_params': self._best_parameters.copy(),
                    'completion_percent': (idx + 1) / self._total_combinations * 100
                }
                self.sigSweepProgress.emit(progress)

                # Save intermediate results
                self._save_intermediate_results()

                # Move to next point
                self._current_sweep_index += 1

            # Sweep completed
            self._handle_completion()

        except Exception as e:
            self.log.error(f'Fatal error in sweep loop: {e}', exc_info=True)
            self.sigError.emit(f'Sweep failed: {str(e)}')
            self._sweep_state = 'idle'
            if self.module_state() == 'locked':
                self.module_state.unlock()

    def _measure_single_point(self, idx: int, params: Dict[str, float]) -> Dict[str, Any]:
        """
        Perform a single sensitivity measurement for given parameters.

        Args:
            idx: Index of current measurement
            params: Parameter dictionary with 'power', 'f_mod', 'f_dev'

        Returns:
            Dictionary with measurement results
        """
        power_dbm = params['power']
        f_mod_hz = params['f_mod']
        f_dev_khz = params['f_dev']

        self.log.info(
            f'\n--- Measurement {idx + 1}/{self._total_combinations} ---\n'
            f'Power: {power_dbm:.2f} dBm, '
            f'f_mod: {f_mod_hz / 1e3:.1f} kHz, '
            f'f_dev: {f_dev_khz:.1f} kHz'
        )

        # Check if power changed (thermal stabilization needed)
        if idx > 0:
            prev_combination = self._parameter_combinations[idx - 1]
            prev_params = dict(zip(self._sweep_loop_order, prev_combination))
            if prev_params['power'] != power_dbm:
                self.log.info(
                    f'Power changed ({prev_params["power"]:.2f} -> {power_dbm:.2f} dBm). '
                    f'Waiting {self._thermal_stabilization_time}s for thermal stabilization...'
                )
                # Use interruptible sleep to allow pause/cancel during thermal wait
                if not self._sleep_interruptible(self._thermal_stabilization_time):
                    self.log.info('Thermal stabilization interrupted by pause/cancel request')
                    raise InterruptedError('Sweep interrupted during thermal stabilization')

        # Create filename nametag for this measurement (just the tag, not full path)
        # Use 'n' prefix for negative power values to avoid '-' in filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        power_str = f'n{abs(power_dbm):.2f}' if power_dbm < 0 else f'{power_dbm:.2f}'
        nametag = f'P_{power_str}dBm_fmod_{f_mod_hz / 1e3:.1f}k_fdev_{f_dev_khz:.1f}k_{timestamp}'

        # Full path prefix for our own data saving (fitting, ASD, etc.)
        filename_prefix = os.path.join(self._current_folder, nametag)

        # Configure lock-in filters ONCE before any measurements
        # This ensures both ODMR scan and time series use the same filter settings
        self._configure_lock_in_filters()

        # Step 1: Configure microwave source
        self._configure_mw_source(power_dbm, f_mod_hz, f_dev_khz)

        # Step 2: Run ODMR scan
        frequencies, odmr_signal = self._run_odmr_scan(nametag)

        # Emit ODMR data for GUI
        self.sigOdmrDataReady.emit(frequencies, odmr_signal)

        # Step 3: Fit resonance
        fit_result = self._fit_odmr_data(frequencies, odmr_signal, filename_prefix)

        if fit_result is None:
            self.log.warning('Fit failed - skipping point')
            return {
                'power_dbm': power_dbm,
                'f_mod_hz': f_mod_hz,
                'f_dev_khz': f_dev_khz,
                'sensitivity_nT_rtHz': np.nan,
                'sensitivity_rms_nT_rtHz': np.nan
            }

        # Emit fit data for GUI (add fit curve data for plotting)
        # Note: fit_hyperfine doesn't return fit_frequency and fit_data arrays
        # We'll construct them from the fitted parabola parameters if needed
        fit_result_with_curves = fit_result.copy()
        # Add empty arrays for now - GUI can plot the original ODMR data
        fit_result_with_curves['fit_frequency'] = frequencies
        fit_result_with_curves['fit_data'] = odmr_signal  # Could compute parabola fit here
        self.sigFitDataReady.emit(fit_result_with_curves)

        # Calculate ODMR center frequency from peak and dip positions
        # Center = mean(mean(peak_positions), mean(dip_positions))
        odmr_center_hz = self._calculate_odmr_center(fit_result)

        # Step 4: Set CW to zero crossing
        zc_freq = fit_result['zero_crossing_frequencies [Hz]'][self._which_zero_crossing]
        slope = fit_result['zero_crossing_slopes [V/Hz]'][self._which_zero_crossing]
        self._set_cw_frequency(zc_freq, power_dbm)

        # Step 5: Record time series and calculate sensitivity
        sensitivity_on, sensitivity_on_rms, asd_freq, asd_on = self._measure_sensitivity(
            filename_prefix + '_ON-resonant',
            slope
        )

        # Emit ASD data for GUI (including sensitivity value for noise floor line)
        self.sigASDDataReady.emit(asd_freq, asd_on, sensitivity_on)

        # Optional: Off-resonant measurement
        # Check stream_params for runtime override, otherwise use ConfigOption
        include_off_resonant = self._stream_parameters.get(
            'include_off_resonant', self._include_off_resonant_measurement
        )
        off_resonant_offset = self._stream_parameters.get(
            'off_resonant_offset_hz', self._off_resonant_offset_hz
        )

        sensitivity_off = np.nan
        sensitivity_off_rms = np.nan
        if include_off_resonant:
            off_freq = zc_freq + off_resonant_offset
            self._set_cw_frequency(off_freq, power_dbm)
            self.log.info(f'Measuring off-resonant sensitivity at {off_freq/1e9:.6f} GHz '
                         f'(+{off_resonant_offset/1e6:.1f} MHz from zero-crossing)')
            sensitivity_off, sensitivity_off_rms, _, _ = self._measure_sensitivity(
                filename_prefix + '_OFF-resonant',
                slope
            )

        # Turn off CW
        self._odmr_logic().toggle_cw_output(False)

        # Compile results
        result = {
            'power_dbm': power_dbm,
            'f_mod_hz': f_mod_hz,
            'f_dev_khz': f_dev_khz,
            'odmr_center_hz': odmr_center_hz,
            'linewidth_hz': fit_result['linewidths [Hz]'][self._which_zero_crossing],
            'zc_slope_V_per_Hz': slope,
            'sensitivity_nT_rtHz': sensitivity_on,
            'sensitivity_rms_nT_rtHz': sensitivity_on_rms,
            'sensitivity_off_resonant_nT_rtHz': sensitivity_off,
            'sensitivity_off_resonant_rms_nT_rtHz': sensitivity_off_rms
        }

        self.log.info(
            f'Measurement complete: Sensitivity = {sensitivity_on:.3f} nT/sqrtHz'
        )

        return result

    def _configure_mw_source(self, power_dbm: float, f_mod_hz: float, f_dev_khz: float):
        """Configure microwave source via ODMR logic."""
        odmr = self._odmr_logic()

        # Turn off CW output first before changing multi-frequency mode
        # The microwave module does not allow mode changes while output is active
        odmr.toggle_cw_output(False)
        time.sleep(0.1)  # Allow settling

        # Set scan power
        odmr.set_scan_power(power_dbm)

        # Get microwave module
        mw = odmr._microwave()

        # Configure multi-frequency mode
        multi_freq_mode = self._odmr_parameters.get('multi_freq_mode', 'triple')
        mw.set_multi_frequency_mode(multi_freq_mode, None)

        # Configure FM parameters
        mw.set_fm_parameters(
            enable=True,
            deviation_khz=f_dev_khz,
            modulation_frequency=f_mod_hz
        )

        self.log.debug(
            f'MW source configured: P={power_dbm} dBm, '
            f'f_mod={f_mod_hz} Hz, f_dev={f_dev_khz} kHz'
        )

    def _run_odmr_scan(self, nametag: str) -> Tuple[np.ndarray, np.ndarray]:
        """Run ODMR scan and return data."""
        odmr = self._odmr_logic()

        # Configure ODMR scan
        odmr.set_runtime(self._odmr_parameters['run_time'])
        odmr.set_frequency_range(
            self._odmr_parameters['frequency_start'],
            self._odmr_parameters['frequency_stop'],
            self._odmr_parameters['frequency_points'],
            0  # no oversampling
        )
        odmr.set_data_rate(self._odmr_parameters.get('data_rate', 1000))

        # Start scan
        self.log.debug('Starting ODMR scan...')
        odmr.start_odmr_scan()

        # Wait for completion with interrupt checking
        run_time = self._odmr_parameters['run_time']
        sleep_interval = min(0.5, run_time / 10) if run_time > 1 else 0.1
        while odmr.module_state() == 'locked':
            # Process events and check for abort
            QtCore.QCoreApplication.processEvents()
            if self._pause_requested or self._cancel_requested:
                self.log.info('ODMR scan interrupted by pause/cancel request')
                odmr.stop_odmr_scan()
                raise InterruptedError('Sweep interrupted during ODMR scan')
            time.sleep(sleep_interval)

        self.log.debug('ODMR scan complete')

        # Save data - pass just the nametag (ODMR logic adds its own path and prefix)
        odmr._save_thumbnails = True
        odmr._use_timestamp = False  # We already include timestamp in nametag
        odmr.save_odmr_data(nametag)

        # Get data
        joined_data = odmr._join_signal_data()
        frequencies = np.array(joined_data.T[0])
        signal = np.array(joined_data.T[1])

        # Normalize (subtract mean)
        signal = signal - np.mean(signal)

        return frequencies, signal

    def _fit_odmr_data(self, frequencies: np.ndarray, signal: np.ndarray,
                      filename_prefix: str) -> Optional[Dict]:
        """Fit ODMR data to extract zero crossings."""
        self.log.debug('Fitting ODMR data...')

        try:
            fit_result = fit_hyperfine(
                frequencies,
                signal,
                feature_prominence=self._min_fit_amplitude,
                n_most_prominent_peaks=self._n_most_prominent_peaks,
                plot_result=False,
                save_result_plot=True,
                min_feature_height=self._min_feature_height,
                filename=filename_prefix
            )

            if fit_result is None:
                return None

            # Check if we have the required zero crossing
            zc_freqs = fit_result.get('zero_crossing_frequencies [Hz]', [])
            if self._which_zero_crossing >= len(zc_freqs):
                self.log.error(
                    f'Fit did not find required zero crossing {self._which_zero_crossing}. '
                    f'Only found {len(zc_freqs)} zero crossings.'
                )
                return None

            self.log.debug(
                f'Fit successful: ZC freq = {zc_freqs[self._which_zero_crossing] / 1e9:.6f} GHz'
            )

            return fit_result

        except Exception as e:
            self.log.error(f'Fitting failed: {e}', exc_info=True)
            return None

    def _calculate_odmr_center(self, fit_result: Dict) -> float:
        """
        Calculate the overall ODMR center frequency from fitted peak and dip positions.

        Args:
            fit_result: Dictionary from fit_hyperfine containing 'peak_positions [Hz]'
                       and 'dip_positions [Hz]' arrays.

        Returns:
            Center frequency in Hz, or np.nan if calculation fails.
        """
        try:
            peak_positions = fit_result.get('peak_positions [Hz]', np.array([]))
            dip_positions = fit_result.get('dip_positions [Hz]', np.array([]))

            # Filter out NaN values
            valid_peaks = peak_positions[~np.isnan(peak_positions)]
            valid_dips = dip_positions[~np.isnan(dip_positions)]

            if len(valid_peaks) == 0 or len(valid_dips) == 0:
                self.log.warning('Cannot calculate ODMR center: no valid peak or dip positions')
                return np.nan

            mean_peak = np.mean(valid_peaks)
            mean_dip = np.mean(valid_dips)
            center = (mean_peak + mean_dip) / 2.0

            self.log.debug(
                f'ODMR center calculation: mean_peak={mean_peak/1e9:.6f} GHz, '
                f'mean_dip={mean_dip/1e9:.6f} GHz, center={center/1e9:.6f} GHz'
            )

            return center

        except Exception as e:
            self.log.error(f'Error calculating ODMR center: {e}')
            return np.nan

    def _set_cw_frequency(self, frequency: float, power: float):
        """Set CW microwave output.

        Note: Must turn CW OFF before changing parameters, because toggle_cw_output(True)
        returns early if CW is already on, without applying the new frequency.
        """
        odmr = self._odmr_logic()

        # Turn off CW first to ensure new parameters are applied
        odmr.toggle_cw_output(False)
        time.sleep(0.1)  # Allow settling

        # Set new parameters
        odmr.set_cw_parameters(float(frequency), float(power))

        # Turn CW back on (this now applies the new frequency)
        odmr.toggle_cw_output(True)
        time.sleep(0.1)  # Allow settling

        self.log.debug(f'CW set to {frequency/1e9:.6f} GHz at {power:.2f} dBm')

    def _configure_lock_in_filters(self):
        """
        Configure lock-in FIR filter settings based on stream_parameters.

        Reads 'fir_bypass' and 'fir_filter_bandwidth' from stream_parameters
        and applies them to the lock-in module via the time_series_logic's streamer.
        """
        fir_bypass = self._stream_parameters.get('fir_bypass', False)
        fir_filter_bw = self._stream_parameters.get('fir_filter_bandwidth', '2kHz_minphase')

        self.log.info(f'Configuring lock-in filters: bypass={fir_bypass}, bandwidth={fir_filter_bw}')

        try:
            # Access the streamer hardware through time_series_logic
            ts_logic = self._time_series_logic()
            streamer = ts_logic._streamer()

            # CRITICAL: Ensure stream input is set to 'demod' for sensitivity measurements
            # This protects against mode changes from other modules (e.g., ODMR tracking
            # might have switched to 'ftw_corr' mode)
            if hasattr(streamer, 'set_stream_input') and hasattr(streamer, 'stream_input'):
                current_mode = streamer.stream_input
                if current_mode != 'demod':
                    self.log.warning(
                        f'Stream input was "{current_mode}", switching to "demod" for sensitivity measurement'
                    )
                    # TSR must be stopped to change mode
                    if ts_logic.module_state() == 'locked':
                        ts_logic.stop_reading()
                    streamer.set_stream_input('demod')
                    self.log.info('Stream input set to "demod"')

            # Access pyrpl instance from streamer
            if not hasattr(streamer, '_pyrpl') or streamer._pyrpl is None:
                self.log.warning('Cannot access pyrpl instance from streamer - lock-in filters not configured')
                return

            pyrpl_instance = streamer._pyrpl

            # Access lockin module (named 'lockin' not 'lock_in' per PyRPL naming convention)
            if not hasattr(pyrpl_instance.rp, 'lockin'):
                self.log.warning('lockin module not available in pyrpl - filters not configured')
                return

            lock_in = pyrpl_instance.rp.lockin

            # Configure FIR bypass
            lock_in.fir_bypass_ch1 = fir_bypass
            self.log.debug(f'Set fir_bypass_ch1 = {fir_bypass}')

            # Configure filter bandwidth (only effective when bypass is False)
            if not fir_bypass:
                # Validate filter bandwidth option
                valid_filters = {'2kHz_minphase', '2kHz_linear', '2kHz'}
                if fir_filter_bw not in valid_filters:
                    self.log.warning(
                        f'Invalid filter selection "{fir_filter_bw}", using "2kHz_minphase"')
                    fir_filter_bw = '2kHz_minphase'

                lock_in.filter_select_ch1 = fir_filter_bw
                self.log.debug(f'Set filter_select_ch1 = {fir_filter_bw}')

            # Verify settings were applied
            actual_bypass = lock_in.fir_bypass_ch1
            actual_filter = lock_in.filter_select_ch1
            self.log.info(f'Lock-in filter configured: bypass={actual_bypass}, filter={actual_filter}')

            if actual_bypass != fir_bypass:
                self.log.error(f'FIR bypass setting mismatch! Requested {fir_bypass}, got {actual_bypass}')
            if not fir_bypass and actual_filter != fir_filter_bw:
                self.log.error(f'Filter bandwidth mismatch! Requested {fir_filter_bw}, got {actual_filter}')

        except AttributeError as e:
            self.log.error(f'Failed to access lock-in module: {e}')
        except Exception as e:
            self.log.error(f'Error configuring lock-in filters: {e}', exc_info=True)

    def _measure_sensitivity(self, filename_prefix: str, slope: float
                            ) -> Tuple[float, float, np.ndarray, np.ndarray]:
        """
        Measure magnetic field sensitivity using time series data.

        Args:
            filename_prefix: Base filename for saving data
            slope: Zero-crossing slope in V/Hz for B-field conversion

        Returns:
            Tuple of (sensitivity_nT_rtHz, sensitivity_rms_nT_rtHz, asd_frequencies, asd_data)
        """
        ts_logic = self._time_series_logic()

        # Get streaming parameters
        n_traces = self._stream_parameters.get('n_time_traces', 32)
        trace_duration = self._stream_parameters.get('trace_duration', 1.0)

        # Get the actual hardware sample rate (Red Pitaya FPGA has fixed ~30.5 kHz)
        actual_sample_rate = ts_logic.sampling_rate
        self.log.info(f'Hardware sample rate: {actual_sample_rate:.1f} Hz')

        # Calculate total samples needed
        # Use hardware sample rate directly (no oversampling)
        samples_per_trace = int(actual_sample_rate * trace_duration)
        total_samples = samples_per_trace * n_traces
        total_duration = n_traces * trace_duration

        self.log.info(
            f'Acquiring sensitivity data: {n_traces} traces × {trace_duration}s = {total_duration}s total, '
            f'{total_samples} samples at {actual_sample_rate:.1f} Hz'
        )

        # Configure time series logic for maximum window
        # Set window size to cover the entire acquisition
        ts_logic.set_trace_settings(
            oversampling_factor=1,
            moving_average_width=1,
            trace_window_size=total_duration,
            data_rate=actual_sample_rate  # Use actual hardware rate
        )

        # IMPORTANT: Stop any existing streaming before starting fresh
        # This ensures a clean state and prevents buffer overflow issues
        if ts_logic.module_state() == 'locked':
            ts_logic.stop_reading()
            time.sleep(0.3)  # Allow stop to complete

        # Start fresh data acquisition
        ts_logic.start_reading()
        time.sleep(1.0)  # Allow buffer to fill initially

        # Wait for the full acquisition time with interrupt checking
        self.log.debug(f'Waiting {total_duration}s for data acquisition...')
        if not self._sleep_interruptible(total_duration + 0.5):
            self.log.info('Data acquisition interrupted by pause/cancel request')
            ts_logic.stop_reading()
            raise InterruptedError('Sweep interrupted during data acquisition')

        # Get trace data from time series logic
        # trace_data returns (times_array, {channel_name: data_array})
        trace_times, trace_data_dict = ts_logic.trace_data

        # Get actual data rate after configuration
        actual_data_rate = ts_logic.data_rate
        self.log.debug(f'Effective data rate: {actual_data_rate:.1f} Hz')

        # Get channel data (get first available channel)
        if not trace_data_dict:
            raise RuntimeError('No channel data available from time_series_logic')

        channel_name = list(trace_data_dict.keys())[0]
        voltage_trace = trace_data_dict[channel_name]

        self.log.info(
            f'Retrieved trace data: {len(voltage_trace)} samples from channel "{channel_name}"'
        )

        # Validate we have enough data
        if len(voltage_trace) < actual_data_rate:
            self.log.warning(
                f'Insufficient data: got {len(voltage_trace)} samples, '
                f'expected at least {actual_data_rate} (1 second worth)'
            )

        # Convert to magnetic field
        # scaling_factor accounts for any analog output scaling
        scaling_factor = 1.0  # Adjust if your hardware has output scaling

        b_field_trace = magnetic_field_from_voltages(
            voltage_trace,
            slope,
            scaling_factor=scaling_factor
        )

        # Calculate actual trace duration from data
        actual_trace_duration = len(voltage_trace) / actual_data_rate
        self.log.debug(f'Actual trace duration: {actual_trace_duration:.3f} s')

        # Ensure trace_times matches the data length
        # time_series_logic may return None or mismatched times
        if trace_times is None or len(trace_times) != len(b_field_trace):
            self.log.debug(f'Reconstructing time array (trace_times was '
                          f'{type(trace_times).__name__}, len={len(trace_times) if trace_times is not None else 0})')
            trace_times = np.arange(len(b_field_trace)) / actual_data_rate

        # Emit time trace for GUI
        self.sigTimeTraceReady.emit(trace_times, b_field_trace)

        # Save raw voltage time trace data as numpy file
        # (conversion factor slope is saved in the summary CSV as zc_slope_V_per_Hz)
        try:
            time_trace_data_path = filename_prefix + '_voltage_time_trace.npz'
            np.savez(
                time_trace_data_path,
                time_s=trace_times,
                voltage_V=voltage_trace,
                sample_rate_Hz=actual_data_rate
            )
            self.log.info(f'Voltage time trace saved: {time_trace_data_path}')
        except Exception as e:
            self.log.error(f'Failed to save time trace data: {e}')

        # Plot and save time traces
        try:
            n_samples = len(b_field_trace)
            self.log.debug(f'Plotting time traces: {n_samples} samples, '
                          f'{len(trace_times)} time points, rate={actual_data_rate:.1f} Hz, '
                          f'duration={actual_trace_duration:.2f} s')
            self.log.debug(f'Time trace filename prefix: {filename_prefix}')

            # Validate we have enough data for plotting
            # The plot function uses int(duration) which would be 0 if duration < 1
            if actual_trace_duration < 1.0:
                self.log.warning(f'Trace duration {actual_trace_duration:.3f}s < 1s, '
                               f'time trace plot may be empty')

            plot_magnetic_field_time_traces(
                b_field_trace,
                trace_times,
                actual_data_rate,
                actual_trace_duration,
                save_fig=True,
                filename_prefix=filename_prefix,
                voltage_trace=voltage_trace
            )

            # Verify file was created
            expected_file = filename_prefix + "_magnetic_field_time_traces.pdf"
            if os.path.exists(expected_file):
                self.log.info(f'Time trace plot saved: {expected_file}')
            else:
                self.log.warning(f'Time trace plot NOT found at expected path: {expected_file}')

        except Exception as e:
            self.log.error(f'Failed to plot time traces: {e}', exc_info=True)

        # Calculate ASD
        # f_ENBW is the Equivalent Noise Bandwidth of the lock-in filter
        # For Red Pitaya IQ demodulation with typical settings, ENBW ~ bandwidth * 1.06
        # Use the config option as default, allow override from stream_params
        f_enbw = self._stream_parameters.get('f_enbw', self._default_f_enbw)
        self.log.debug(f'Using f_ENBW = {f_enbw:.1f} Hz for ASD calculation')

        # Get sensitivity bandwidth parameters from stream_params
        sensitivity_f_min = self._stream_parameters.get('sensitivity_f_min', 200.0)
        sensitivity_f_max = self._stream_parameters.get('sensitivity_f_max', 1400.0)
        exclude_50hz = self._stream_parameters.get('exclude_50hz_harmonics', True)
        self.log.debug(f'Sensitivity bandwidth: {sensitivity_f_min:.0f}-{sensitivity_f_max:.0f} Hz, '
                      f'exclude 50Hz harmonics: {exclude_50hz}')

        try:
            asd_result = plot_asds(
                b_field_trace,
                actual_data_rate,
                actual_trace_duration,
                f_enbw,  # 4th positional argument: f_ENBW
                save_fig=True,
                save_data=True,
                filename_prefix=filename_prefix,
                sensitivity_f_min=sensitivity_f_min,
                sensitivity_f_max=sensitivity_f_max,
                exclude_50hz_harmonics=exclude_50hz
            )

            sensitivity = asd_result['sensitivity']
            sensitivity_rms = asd_result.get('sensitivity_rms', np.nan)
            asd_frequencies = asd_result.get('frequencies', np.array([]))
            asd_data = asd_result.get('asd_hanning', np.array([]))  # Use Hanning window ASD

            self.log.info(f'Sensitivity: {sensitivity:.3f} nT/sqrt(Hz) (RMS: {sensitivity_rms:.3f})')

        except Exception as e:
            self.log.error(f'ASD calculation failed: {e}', exc_info=True)
            sensitivity = np.nan
            sensitivity_rms = np.nan
            asd_frequencies = np.array([])
            asd_data = np.array([])

        # CRITICAL: Stop streaming before returning to avoid concurrent Red Pitaya access
        # when other operations (like toggle_cw_output) try to communicate with the FPGA
        try:
            if ts_logic.module_state() == 'locked':
                ts_logic.stop_reading()
                time.sleep(0.5)  # Allow streaming thread to fully stop
                self.log.debug('Stopped time series streaming')
        except Exception as e:
            self.log.warning(f'Failed to stop streaming cleanly: {e}')

        return sensitivity, sensitivity_rms, asd_frequencies, asd_data

    # =========================================================================
    # Internal Methods - State Management
    # =========================================================================

    def _handle_pause(self):
        """Handle pause request."""
        with self._thread_lock:
            self._sweep_state = 'paused'
            self._pause_requested = False

            self.sigSweepPaused.emit()
            self.log.info(f'Sweep paused at point {self._current_sweep_index + 1}')

            if self.module_state() == 'locked':
                self.module_state.unlock()

    def _handle_cancel(self):
        """Handle cancel request."""
        with self._thread_lock:
            self._sweep_state = 'cancelled'
            self._cancel_requested = False

            # Save results collected so far
            self._save_final_results()

            # Generate summary visualization plots (even for cancelled sweeps)
            self._generate_summary_plots()

            self.sigSweepCancelled.emit()
            self.log.info(f'Sweep cancelled after {self._current_sweep_index} measurements')

            if self.module_state() == 'locked':
                self.module_state.unlock()

    def _handle_completion(self):
        """Handle sweep completion."""
        with self._thread_lock:
            self._sweep_state = 'idle'

            # Save final results
            self._save_final_results()

            # Generate summary visualization plots
            self._generate_summary_plots()

            # Emit completion signal
            self.sigSweepFinished.emit(self._current_folder)

            self.log.info(
                f'\n=== Sweep Complete ===\n'
                f'Total measurements: {len(self._results_list)}\n'
                f'Best sensitivity: {self._best_sensitivity:.3f} nT/sqrtHz\n'
                f'Best parameters: {self._best_parameters}\n'
                f'Results saved to: {self._current_folder}'
            )

            if self.module_state() == 'locked':
                self.module_state.unlock()

    # =========================================================================
    # Internal Methods - Data Management
    # =========================================================================

    def _create_measurement_folder(self):
        """Create timestamped folder for measurement data."""
        base_folder = self.module_default_data_dir
        timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        folder_name = os.path.join(base_folder, 'SensitivitySweep', timestamp)

        if not os.path.exists(folder_name):
            os.makedirs(folder_name)

        self._current_folder = folder_name
        self.log.info(f'Data folder: {folder_name}')

    def _save_intermediate_results(self):
        """Save intermediate results (called after each point)."""
        if not self._results_list:
            return

        try:
            df = pd.DataFrame(self._results_list)
            csv_path = os.path.join(self._current_folder, 'parameter_sweep_summary.csv')
            df.to_csv(csv_path, sep='\t', index=False, na_rep='NaN')
        except Exception as e:
            self.log.error(f'Error saving intermediate results: {e}')

    def _save_final_results(self):
        """Save final results and metadata."""
        try:
            # Save results DataFrame
            if self._results_list:
                df = pd.DataFrame(self._results_list)
                csv_path = os.path.join(self._current_folder, 'parameter_sweep_summary.csv')
                df.to_csv(csv_path, sep='\t', index=False, na_rep='NaN')

            # Save metadata
            # Determine the effective off-resonant settings (from stream_params or ConfigOptions)
            include_off_resonant = self._stream_parameters.get(
                'include_off_resonant', self._include_off_resonant_measurement
            )
            off_resonant_offset = self._stream_parameters.get(
                'off_resonant_offset_hz', self._off_resonant_offset_hz
            )

            # Get sensitivity bandwidth parameters
            sensitivity_f_min = self._stream_parameters.get('sensitivity_f_min', 200.0)
            sensitivity_f_max = self._stream_parameters.get('sensitivity_f_max', 1400.0)
            exclude_50hz = self._stream_parameters.get('exclude_50hz_harmonics', True)

            metadata = {
                'total_measurements': len(self._results_list),
                'total_planned': self._total_combinations,
                'best_sensitivity_nT_rtHz': float(self._best_sensitivity),
                'best_parameters': {k: float(v) for k, v in self._best_parameters.items()},
                'sweep_loop_order': self._sweep_loop_order,
                'thermal_stabilization_time_s': self._thermal_stabilization_time,
                'which_zero_crossing': self._which_zero_crossing,
                'odmr_parameters': self._odmr_parameters,
                'stream_parameters': self._stream_parameters,
                'include_off_resonant_measurement': include_off_resonant,
                'off_resonant_offset_hz': off_resonant_offset,
                'sensitivity_bandwidth_hz': [sensitivity_f_min, sensitivity_f_max],
                'exclude_50hz_harmonics': exclude_50hz,
                'timestamp': datetime.now().isoformat()
            }

            json_path = os.path.join(self._current_folder, 'sweep_metadata.json')
            with open(json_path, 'w') as f:
                json.dump(metadata, f, indent=4)

            self.log.info(f'Results saved to {self._current_folder}')

        except Exception as e:
            self.log.error(f'Error saving final results: {e}', exc_info=True)

    def _generate_summary_plots(self):
        """
        Generate summary visualization plots for the sweep results.

        Creates plots showing how sensitivity and ODMR parameters vary with
        the swept parameters. Plots are saved to a 'summary_plots' subfolder.
        """
        if not self._results_list or len(self._results_list) < 2:
            self.log.info('Not enough data points for summary visualization (need >= 2)')
            return

        try:
            # Prepare data and metadata
            df = pd.DataFrame(self._results_list)

            # Build metadata dict with current state
            metadata = {
                'best_sensitivity_nT_rtHz': float(self._best_sensitivity),
                'best_parameters': {k: float(v) for k, v in self._best_parameters.items()},
                'sweep_loop_order': self._sweep_loop_order,
                'total_measurements': len(self._results_list),
                'total_planned': self._total_combinations
            }

            # Create visualizer and generate plots
            visualizer = SensitivitySweepVisualizer(
                results_df=df,
                metadata=metadata,
                output_folder=self._current_folder,
                logger=self.log
            )

            generated_files = visualizer.generate_all_plots()

            if generated_files:
                self.log.info(
                    f'Generated {len(generated_files)} summary plots in '
                    f'{os.path.join(self._current_folder, "summary_plots")}'
                )
            else:
                self.log.warning('No summary plots were generated')

        except Exception as e:
            self.log.error(f'Error generating summary plots: {e}', exc_info=True)
