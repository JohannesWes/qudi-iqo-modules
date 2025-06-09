"""
This file contains the Qudi hardware module for a combined microwave source consisting of:
- Red Pitaya as IF source (with IQ calibration and optional FM)
- IQ Mixer for upconversion
- Windfreak SynthNV Pro as LO source

The actual output frequency is: RF = LO - IF
For multi-frequency excitation: RF_i = LO - IF_i for each IF frequency
"""

import time
import pyvisa
import numpy as np
from typing import Optional, Dict, List, Union
import sys
import os

from qudi.util.mutex import Mutex
from qudi.core.configoption import ConfigOption
from qudi.interface.microwave_interface import MicrowaveInterface, MicrowaveConstraints
from qudi.util.enums import SamplingOutputMode

from .redpitaya.if_source_base import IFSourceBase
from .redpitaya.redpitaya_if_source import RedPitayaIFSource


class MicrowaveRedPitayaWindfreak(MicrowaveInterface):
    """ Hardware class to control a combined microwave source:
        - Red Pitaya as IF source (with IQ calibration and optional FM)
        - IQ Mixer for upconversion
        - Windfreak SynthNV Pro as LO source

    The actual RF output frequency is: RF = LO - IF
    For multi-frequency excitation: RF_i = LO - IF_i for each IF frequency

    Example config for copy-paste:

    mw_source_rp_windfreak:
        module.Class: 'microwave.mw_source_redpitaya_windfreak.MicrowaveRedPitayaWindfreak'
        options:
            windfreak_serial_port: 'COM3'
            windfreak_comm_timeout: 10  # in seconds
            redpitaya_hostname: '10.203.129.28'
            if_frequencies: [19.422e6, 21.580e6, 23.738e6]  # Hz
            if_frequency_index: 1  # Use middle frequency (21.580 MHz) for calculations
            lo_power: 13  # dBm - fixed power for IQ mixer LO input
            calibration_files:
                19.422e6: 'path/to/calibration_IF_19.422MHz.csv'
                21.580e6: 'path/to/calibration_IF_21.580MHz.csv'
                23.738e6: 'path/to/calibration_IF_23.738MHz.csv'
            enable_fm: False  # Enable FM modulation capability
            fm_deviation_khz: 100.0  # Default FM deviation in kHz
            fm_modulation_frequency: 5000.0  # Default FM modulation frequency in Hz
            power_calibration_table: null  # Optional: path to power calibration file
            multi_frequency_mode: 'single'  # Options: 'single', 'dual', 'triple'
            multi_frequency_amplitudes: [0.5, 0.5, 0.5]  # Relative amplitudes for multi-freq mode
    """

    # Windfreak config options
    _windfreak_serial_port = ConfigOption('windfreak_serial_port', missing='error')
    _windfreak_comm_timeout = ConfigOption('windfreak_comm_timeout', default=10, missing='warn')

    # Red Pitaya config options
    _redpitaya_hostname = ConfigOption('redpitaya_hostname', missing='error')
    _redpitaya_port = ConfigOption('redpitaya_port', default=2222, missing='info')

    # IF configuration
    _if_frequencies = ConfigOption('if_frequencies', missing='error')
    _if_frequency_index = ConfigOption('if_frequency_index', default=1, missing='info')
    _lo_power = ConfigOption('lo_power', default=13, missing='info')
    _calibration_files = ConfigOption('calibration_files', missing='error')

    # FM configuration
    _enable_fm = ConfigOption('enable_fm', default=False, missing='info')
    _fm_deviation_khz = ConfigOption('fm_deviation_khz', default=100.0, missing='info')
    _fm_modulation_frequency = ConfigOption('fm_modulation_frequency', default=5000.0, missing='info')

    # Power calibration
    _power_calibration_table = ConfigOption('power_calibration_table', default=None, missing='info')

    # Multi-frequency configuration
    _multi_frequency_mode = ConfigOption('multi_frequency_mode', default='single', missing='info')
    _multi_frequency_amplitudes = ConfigOption('multi_frequency_amplitudes', default=[0.5, 0.5, 0.5], missing='info')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = Mutex()

        # Windfreak resources
        self._rm = None
        self._windfreak_device = None
        self._windfreak_model = ''

        # Red Pitaya instance
        self._redpitaya = None

        # Constraints
        self._constraints = None

        # State variables
        self._scan_power = -20
        self._scan_mode = SamplingOutputMode.EQUIDISTANT_SWEEP
        self._scan_frequencies = None
        self._scan_sample_rate = 0.
        self._scan_step_time = 0.
        self._in_cw_mode = True

        # IF frequency for calculations (from config)
        self._if_frequency = None
        self._active_if_frequencies = []
        self._active_if_amplitudes = []

        # Current settings
        self._current_rf_frequency = 2.87e9  # Hz
        self._current_rf_power = -20  # dBm

        # Power calibration data
        self._power_cal_data = None

        # Initialize per-component FM settings
        self._fm_enables_per_component = []
        self._fm_deviations_per_component = []

    def on_activate(self):
        """ Initialisation performed during activation of the module. """
        try:
            # Set up IF frequency from config
            self._if_frequency = self._if_frequencies[self._if_frequency_index]
            self.log.info(f'Using IF frequency: {self._if_frequency / 1e6:.3f} MHz for calculations')

            # Initialize multi-frequency settings
            self._update_active_frequencies()

            # Initialize per-component FM settings
            self._fm_enables_per_component = [self._enable_fm] * len(self._if_frequencies)
            self._fm_deviations_per_component = [self._fm_deviation_khz] * len(self._if_frequencies)

            # Load power calibration if available
            if self._power_calibration_table:
                try:
                    import pandas as pd
                    self._power_cal_data = pd.read_csv(self._power_calibration_table)
                    self.log.info(f'Loaded power calibration from {self._power_calibration_table}')
                except Exception as e:
                    self.log.warning(f'Could not load power calibration: {e}')

            # Connect to Windfreak
            self._rm = pyvisa.ResourceManager()
            self._windfreak_device = self._rm.open_resource(
                self._windfreak_serial_port,
                baud_rate=9600,
                read_termination='\n',
                write_termination='\n',
                timeout=int(self._windfreak_comm_timeout * 1000)
            )
            self._windfreak_model = self._windfreak_device.query('+')
            self.log.info(f'Connected to Windfreak: {self._windfreak_model}')

            # Connect to Red Pitaya
            self._redpitaya = RedPitayaIFSource(self._redpitaya_hostname, self._redpitaya_port)
            self._redpitaya.connect()
            self.log.info('Connected to Red Pitaya')

            # Load calibration data
            for freq, cal_file in self._calibration_files.items():
                try:
                    self._redpitaya.load_calibration_data(freq, cal_file)
                    self.log.info(f'Loaded calibration for {freq / 1e6:.3f} MHz')
                except Exception as e:
                    self.log.error(f'Failed to load calibration for {freq / 1e6:.3f} MHz: {e}')

            # Generate constraints based on average IF frequency
            avg_if_freq = self._get_average_if_frequency()
            min_rf = 100e6  # Set reasonable minimum RF frequency (100 MHz)
            max_rf = 6.4e9 - avg_if_freq

            self._constraints = MicrowaveConstraints(
                power_limits=(-50, 0),  # Limited by IF source dynamic range
                frequency_limits=(min_rf, max_rf),
                scan_size_limits=(2, 10000),
                sample_rate_limits=(0.1, 2500),
                scan_modes=(SamplingOutputMode.EQUIDISTANT_SWEEP, SamplingOutputMode.JUMP_LIST)
            )

            self._scan_power = -20
            self._scan_frequencies = None
            self._scan_sample_rate = self._constraints.max_sample_rate
            self._in_cw_mode = True

        except Exception as e:
            self.log.error(f'Failed to activate module: {e}')
            raise

    def on_deactivate(self):
        """ Cleanup performed during deactivation of the module. """
        try:
            self.off()

            # Disconnect Red Pitaya
            if self._redpitaya:
                self._redpitaya.disconnect()
                self._redpitaya = None

            # Disconnect Windfreak
            if self._windfreak_device:
                self._windfreak_device.close()
                self._windfreak_device = None
            if self._rm:
                self._rm.close()
                self._rm = None

        except Exception as e:
            self.log.error(f'Error during deactivation: {e}')

    def _update_active_frequencies(self):
        """Update the active IF frequencies based on multi-frequency mode."""
        if self._multi_frequency_mode == 'single':
            self._active_if_frequencies = [self._if_frequency]
            self._active_if_amplitudes = [1.0]
        elif self._multi_frequency_mode == 'dual':
            # Use two frequencies symmetrically around the center frequency
            if len(self._if_frequencies) >= 2:
                self._active_if_frequencies = [self._if_frequencies[0], self._if_frequencies[2]] if len(self._if_frequencies) > 2 else self._if_frequencies[:2]
                self._active_if_amplitudes = self._multi_frequency_amplitudes[:2]
            else:
                self.log.warning('Not enough IF frequencies for dual mode, falling back to single')
                self._active_if_frequencies = [self._if_frequency]
                self._active_if_amplitudes = [1.0]
        elif self._multi_frequency_mode == 'triple':
            # Use all three frequencies
            if len(self._if_frequencies) >= 3:
                self._active_if_frequencies = self._if_frequencies[:3]
                self._active_if_amplitudes = self._multi_frequency_amplitudes[:3]
            else:
                self.log.warning('Not enough IF frequencies for triple mode, falling back to single')
                self._active_if_frequencies = [self._if_frequency]
                self._active_if_amplitudes = [1.0]
        else:
            self.log.error(f'Unknown multi-frequency mode: {self._multi_frequency_mode}')
            self._active_if_frequencies = [self._if_frequency]
            self._active_if_amplitudes = [1.0]

        # Normalize amplitudes
        total_amp = sum(self._active_if_amplitudes)
        if total_amp > 0:
            self._active_if_amplitudes = [a / total_amp for a in self._active_if_amplitudes]

    def _get_average_if_frequency(self):
        """Calculate the average of active IF frequencies."""
        return np.mean(self._active_if_frequencies)

    def set_multi_frequency_mode(self, mode: str, amplitudes: Optional[List[float]] = None):
        """Set the multi-frequency mode.

        @param str mode: 'single', 'dual', or 'triple'
        @param List[float] amplitudes: Optional relative amplitudes for each frequency
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to change multi-frequency mode. Microwave output active.')

            if mode not in ['single', 'dual', 'triple']:
                raise ValueError(f'Invalid mode: {mode}. Must be "single", "dual", or "triple"')

            self._multi_frequency_mode = mode
            if amplitudes is not None:
                self._multi_frequency_amplitudes = amplitudes

            self._update_active_frequencies()

            # Update constraints based on new average IF frequency
            avg_if_freq = self._get_average_if_frequency()
            min_rf = 100e6
            max_rf = 6.4e9 - avg_if_freq

            self._constraints = MicrowaveConstraints(
                power_limits=(-50, 0),
                frequency_limits=(min_rf, max_rf),
                scan_size_limits=(2, 10000),
                sample_rate_limits=(0.1, 2500),
                scan_modes=(SamplingOutputMode.EQUIDISTANT_SWEEP, SamplingOutputMode.JUMP_LIST)
            )

            self.log.info(
                f'Set multi-frequency mode to {mode} with {len(self._active_if_frequencies)} active frequencies')

    @property
    def constraints(self):
        return self._constraints

    @property
    def is_scanning(self):
        """Read-Only boolean flag indicating if a scan is running at the moment. Can be used together with
        module_state() to determine if the currently running microwave output is a scan or CW.
        Should return False if module_state() is 'idle'.

        @return bool: Flag indicating if a scan is running (True) or not (False)
        """
        with self._thread_lock:
            return (self.module_state() != 'idle') and not self._in_cw_mode

    @property
    def cw_power(self):
        """The CW microwave power in dBm. Must implement setter as well.

        @return float: The currently set CW microwave power in dBm.
        """
        with self._thread_lock:
            return self._current_rf_power

    @property
    def cw_frequency(self):
        """The CW microwave frequency in Hz. Must implement setter as well.

        @return float: The currently set CW microwave frequency in Hz.
        """
        with self._thread_lock:
            # Return the center RF frequency based on average IF
            avg_if_freq = self._get_average_if_frequency()
            return float(self._windfreak_device.query('f?')) * 1e6 - avg_if_freq

    @property
    def scan_power(self):
        """The microwave power in dBm used for scanning. Must implement setter as well.

        @return float: The currently set scanning microwave power in dBm
        """
        with self._thread_lock:
            return self._scan_power

    @property
    def scan_frequencies(self):
        """The microwave frequencies used for scanning. Must implement setter as well.

        In case of scan_mode == SamplingOutputMode.JUMP_LIST, this will be a 1D numpy array.
        In case of scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP, this will be a tuple
        containing 3 values (freq_begin, freq_end, number_of_samples).
        If no frequency scan has been specified, return None.

        @return float[]: The currently set scanning frequencies. None if not set.
        """
        with self._thread_lock:
            return self._scan_frequencies

    @property
    def scan_mode(self):
        """Scan mode Enum. Must implement setter as well.

        @return SamplingOutputMode: The currently set scan mode Enum
        """
        with self._thread_lock:
            return self._scan_mode

    @property
    def scan_sample_rate(self):
        """Read-only property returning the currently configured scan sample rate in Hz.

        @return float: The currently set scan sample rate in Hz
        """
        with self._thread_lock:
            return self._scan_sample_rate

    def set_cw(self, frequency, power):
        """Configure the CW microwave output. Does not start physical signal output, see also
        "cw_on".

        @param float frequency: RF frequency to set in Hz (center frequency for multi-freq mode)
        @param float power: RF power to set in dBm (total power of all IF components)
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to set CW parameters. Microwave output active.')
            self._assert_cw_parameters_args(frequency, power)

            self._current_rf_frequency = frequency
            self._current_rf_power = power

            # Calculate required LO frequency based on average IF
            avg_if_freq = self._get_average_if_frequency()
            lo_frequency = frequency + avg_if_freq

            # Configure Windfreak for CW at calculated LO frequency
            self._windfreak_device.write('X0')  # sweep mode off
            self._windfreak_device.write('c1')  # continuous mode
            self._windfreak_device.write('y0')  # trigger mode: software
            self._windfreak_device.write(f'f{lo_frequency / 1e6:5.7f}')
            self._windfreak_device.write(f'l{lo_frequency / 1e6:5.7f}')
            self._windfreak_device.write(f'u{lo_frequency / 1e6:5.7f}')

            # Calculate IF amplitudes for each frequency component
            if_amplitudes = []
            for amp_ratio in self._active_if_amplitudes:
                if_amplitude = self._power_to_if_amplitude(power) * amp_ratio
                if_amplitudes.append(if_amplitude)

            # Use per-component FM settings if available
            fm_enables = getattr(self, '_fm_enables_per_component',
                                 [self._enable_fm] * len(self._active_if_frequencies))
            fm_deviations = getattr(self, '_fm_deviations_per_component',
                                    [self._fm_deviation_khz] * len(self._active_if_frequencies))

            # Only use FM settings for active components
            fm_enables = fm_enables[:len(self._active_if_frequencies)]
            fm_deviations = fm_deviations[:len(self._active_if_frequencies)]

            fm_mod_freq = self._fm_modulation_frequency if any(fm_enables) else None

            # Build calibration files dict for active frequencies
            active_cal_files = {freq: self._calibration_files[freq]
                                for freq in self._active_if_frequencies
                                if freq in self._calibration_files}

            # Use the calibrated multi-frequency setup
            self._redpitaya.set_multi_frequency_signal(
                frequencies=self._active_if_frequencies,
                amplitudes=if_amplitudes,
                lo_frequency=lo_frequency,
                calibration_files=active_cal_files,
                fm_enables=fm_enables,
                fm_deviations_khz=fm_deviations,
                fm_modulation_frequency=fm_mod_freq
            )

            self.log.debug(f'CW configured: RF center={frequency / 1e9:.4f} GHz, LO={lo_frequency / 1e9:.4f} GHz, '
                           f'Active IFs={[f / 1e6 for f in self._active_if_frequencies]} MHz, Power={power} dBm, '
                           f'Mode={self._multi_frequency_mode}, FM enables={fm_enables}')

    def configure_scan(self, power, frequencies, mode, sample_rate):
        """Configure frequency scan."""
        with self._thread_lock:
            # Sanity checks
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to configure frequency scan. Microwave output active.')
            self._assert_scan_configuration_args(power, frequencies, mode, sample_rate)

            self._scan_power = power
            self._scan_mode = mode
            self._scan_sample_rate = sample_rate

            # Calculate IF amplitudes for each frequency component
            if_amplitudes = []
            for amp_ratio in self._active_if_amplitudes:
                if_amplitude = self._power_to_if_amplitude(power) * amp_ratio
                if_amplitudes.append(if_amplitude)

            # Use per-component FM settings
            fm_enables = getattr(self, '_fm_enables_per_component',
                                 [self._enable_fm] * len(self._active_if_frequencies))
            fm_deviations_khz = getattr(self, '_fm_deviations_per_component',
                                        [self._fm_deviation_khz] * len(self._active_if_frequencies))

            # Only use FM settings for active components
            fm_enables = fm_enables[:len(self._active_if_frequencies)]
            fm_deviations_khz = fm_deviations_khz[:len(self._active_if_frequencies)]

            fm_mod_freq = self._fm_modulation_frequency if any(fm_enables) else None

            # Calculate average IF frequency for LO calculations
            avg_if_freq = self._get_average_if_frequency()

            if mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                # For sweep mode, calculate LO frequencies
                rf_start, rf_stop, num_points = frequencies
                lo_start = rf_start + avg_if_freq
                lo_stop = rf_stop + avg_if_freq

                # Use mid-point LO frequency for calibration
                lo_mid = (lo_start + lo_stop) / 2

                # Build calibration files dict for active frequencies
                active_cal_files = {freq: self._calibration_files[freq]
                                    for freq in self._active_if_frequencies
                                    if freq in self._calibration_files}

                # Configure Red Pitaya with multi-frequency signal
                self._redpitaya.set_multi_frequency_signal(
                    frequencies=self._active_if_frequencies,
                    amplitudes=if_amplitudes,
                    lo_frequency=lo_mid,
                    calibration_files=active_cal_files,
                    fm_enables=fm_enables,
                    fm_deviations_khz=fm_deviations_khz,
                    fm_modulation_frequency=fm_mod_freq
                )

                # Configure Windfreak for sweep
                self._configure_windfreak_sweep(lo_start, lo_stop, num_points, sample_rate)
                self._scan_frequencies = tuple(frequencies)

            elif mode == SamplingOutputMode.JUMP_LIST:
                # For jump list, calculate all LO frequencies
                rf_frequencies = np.asarray(frequencies, dtype=np.float64)
                lo_frequencies = rf_frequencies + avg_if_freq

                # Use mid-point LO frequency for calibration
                lo_mid = np.mean([np.min(lo_frequencies), np.max(lo_frequencies)])

                # Build calibration files dict for active frequencies
                active_cal_files = {freq: self._calibration_files[freq]
                                    for freq in self._active_if_frequencies
                                    if freq in self._calibration_files}

                # Configure Red Pitaya with multi-frequency signal
                self._redpitaya.set_multi_frequency_signal(
                    frequencies=self._active_if_frequencies,
                    amplitudes=if_amplitudes,
                    lo_frequency=lo_mid,
                    calibration_files=active_cal_files,
                    fm_enables=fm_enables,
                    fm_deviations_khz=fm_deviations_khz,
                    fm_modulation_frequency=fm_mod_freq
                )

                # Configure Windfreak for jump list
                self._configure_windfreak_list(lo_frequencies, sample_rate)
                self._scan_frequencies = rf_frequencies

            # Wait for configuration to settle
            time.sleep(0.2)

            self.log.debug(f'Configured scan: mode={mode}, power={power} dBm, '
                           f'sample_rate={sample_rate} Hz, multi_freq_mode={self._multi_frequency_mode}, '
                           f'FM enables={fm_enables}')

    def off(self):
        """Switches off any microwave output (both scan and CW)."""
        with self._thread_lock:
            if self.module_state() != 'idle':
                # Turn off Windfreak
                self._windfreak_off()
                # Turn off Red Pitaya
                self._redpitaya.enable_output(False)
                self.module_state.unlock()
                self.log.debug('All outputs turned off')

    def cw_on(self):
        """Switches on cw microwave output."""
        with self._thread_lock:
            if self.module_state() != 'idle':
                if self._in_cw_mode:
                    return
                raise RuntimeError('Unable to start CW microwave output. Microwave output is currently active.')

            self._in_cw_mode = True

            # Enable Red Pitaya output first
            self._redpitaya.enable_output(True)
            time.sleep(0.1)  # Small delay to ensure IF is stable

            # Enable Windfreak
            self._windfreak_on()

            # For Windfreak, ensure we're in the right mode
            self._windfreak_device.write('g1')  # Enable generator

            self.module_state.lock()
            self.log.debug('CW output enabled')

    def start_scan(self):
        """Switches on the microwave scanning.

        Must return AFTER the output is actually active (and can receive triggers for example).
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if not self._in_cw_mode:
                    return
                raise RuntimeError('Unable to start frequency scan. CW microwave output is active.')

            assert self._scan_frequencies is not None, 'No scan_frequencies set. Unable to start scan.'

            self._in_cw_mode = False

            # Enable Red Pitaya output
            self._redpitaya.enable_output(True)
            time.sleep(0.1)  # Small delay to ensure IF is stable

            # Enable Windfreak output
            self._windfreak_on()

            # Start Windfreak scan
            if self._scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                self._windfreak_device.write('g1g0')  # Enable and reset sweep
            # For jump list, the list is already configured and will start on trigger

            self.module_state.lock()
            self.log.debug('Scan started')

    def reset_scan(self):
        """Reset currently running scan and return to start frequency.
        Does not need to stop and restart the microwave output if the device allows soft scan reset.
        """
        with self._thread_lock:
            if self.module_state() == 'idle':
                return
            if self._in_cw_mode:
                raise RuntimeError('Can not reset frequency scan. CW microwave output active.')

            if self._scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                # Reset Windfreak sweep
                self._windfreak_device.write('g1g0')
            else:
                # For jump list mode
                time.sleep(self._scan_step_time)
                self._windfreak_device.write('X1')
                self._windfreak_device.write('g0')

            self.log.debug('Scan reset')

    def _configure_windfreak_sweep(self, lo_start, lo_stop, points, sample_rate):
        """Configure Windfreak for sweep mode."""
        step = (lo_stop - lo_start) / (points - 1)

        # Set step time
        step_time_ms = 1000 * 0.75 / sample_rate
        self._windfreak_device.write(f't{step_time_ms:f}')
        self._scan_step_time = 0.75 / sample_rate

        # Disable temperature compensation (Windfreak bug workaround)
        self._windfreak_device.write('Z0')

        # Configure sweep parameters
        self._windfreak_device.write('X0')  # sweep mode: linear sweep
        self._windfreak_device.write('c0')  # non-continuous
        self._windfreak_device.write('y2')  # trigger mode: single step

        # Set sweep direction
        if lo_stop >= lo_start:
            self._windfreak_device.write('^1')
        else:
            self._windfreak_device.write('^0')

        # Set frequencies
        self._windfreak_device.write(f'l{lo_start / 1e6:5.7f}')
        self._windfreak_device.write(f'u{lo_stop / 1e6:5.7f}')
        self._windfreak_device.write(f's{step / 1e6:5.7f}')

        # Set power (constant LO power)
        self._windfreak_device.write(f'W{self._lo_power:2.3f}')
        self._windfreak_device.write(f'[{self._lo_power:2.3f}')  # sweep lower power
        self._windfreak_device.write(f']{self._lo_power:2.3f}')  # sweep upper power

    def _configure_windfreak_list(self, lo_frequencies, sample_rate):
        """Configure Windfreak for jump list mode."""
        # Set step time
        step_time_ms = 1000 * 0.75 / sample_rate
        self._windfreak_device.write(f't{step_time_ms:f}')
        self._scan_step_time = 0.75 / sample_rate

        # Disable temperature compensation
        self._windfreak_device.write('Z0')

        # Configure for tabular sweep
        self._windfreak_device.write('c0')  # non-continuous
        self._windfreak_device.write('X1')  # tabular sweep mode
        self._windfreak_device.write('y2')  # trigger mode: single step

        # Delete old list
        self._windfreak_device.write('Ld')

        # Create frequency list with constant LO power
        list_strings = []
        for i, freq in enumerate(lo_frequencies):
            list_strings.append(f"L{i}f{freq / 1e6:.6f}L{i}a{self._lo_power}")

        # Write list in chunks due to VISA limitations
        chunk_size = 175
        for i in range(0, len(list_strings), chunk_size):
            chunk = "".join(list_strings[i:i + chunk_size])
            self._windfreak_device.write(chunk)
            time.sleep(0.1)

        # Reset to beginning of list
        self._windfreak_device.write('X1')
        self._windfreak_device.write('g0')

    def _windfreak_off(self):
        """Turn off Windfreak output."""
        # disable sweep mode
        self._windfreak_device.write('g0')
        # set trigger source to software
        self._windfreak_device.write('y0')
        self._windfreak_device.write('E0h0')
        return self._windfreak_stat()

    def _windfreak_on(self):
        """Turn on Windfreak output."""
        self._windfreak_device.write(f'W{self._lo_power}')
        self._windfreak_device.write('E1h1')
        return self._windfreak_stat()

    def _windfreak_stat(self):
        """Return Windfreak status."""
        try:
            E = int(self._windfreak_device.query('E?'))
            h = int(self._windfreak_device.query('h?'))
            return E, h
        except Exception as e:
            self.log.error(f'Failed to get Windfreak status: {e}')
            return 0, 0

    def _power_to_if_amplitude(self, power_dbm):
        """Convert desired RF power in dBm to IF amplitude / peak voltage (0-1) V.

        This uses either a calibration table or formula for 50 Ohm system.
        """
        if self._power_cal_data is not None:
            try:
                # Interpolate from calibration data
                # Assuming calibration data has columns: 'power_dbm', 'if_amplitude'
                return np.interp(power_dbm,
                                 self._power_cal_data['power_dbm'],
                                 self._power_cal_data['if_amplitude'])
            except Exception as e:
                self.log.warning(f'Error using power calibration data: {e}')

        if_amplitude = 10 ** ((power_dbm - 10) / 20)

        return if_amplitude

    def _if_amplitude_to_power(self, if_amplitude):
        """Convert IF amplitude / peak voltage (0-1) V to RF power in dBm.

        This uses either a calibration table or a formula for 50 Ohm system.
        """
        if self._power_cal_data is not None:
            try:
                # Interpolate from calibration data
                return np.interp(if_amplitude,
                                 self._power_cal_data['if_amplitude'],
                                 self._power_cal_data['power_dbm'])
            except Exception as e:
                self.log.warning(f'Error using power calibration data: {e}')

        power_dbm = 20 * np.log10(if_amplitude) + 10

        return power_dbm

    def set_component_enabled(self, component_index: int, enabled: bool):
        """Enable or disable a specific frequency component.

        @param int component_index: Index of the component (0-2)
        @param bool enabled: True to enable, False to disable
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to change component state. Microwave output active.')

            if component_index >= len(self._if_frequencies):
                raise ValueError(f'Component index {component_index} out of range')

            # This would need to be implemented with a new data structure
            # to track which components are enabled
            # For now, we'll implement this through the Red Pitaya
            if self._redpitaya and self._redpitaya.is_connected:
                config = self._redpitaya.get_current_config()
                if config and component_index < len(config.components):
                    config.components[component_index].enabled = enabled
                    self._redpitaya.configure_signal(config)

    def set_fm_parameters(self, enable=None, deviation_khz=None, modulation_frequency=None):
        """Set FM modulation parameters.

        @param bool enable: Enable/disable FM modulation
        @param float deviation_khz: FM deviation in kHz
        @param float modulation_frequency: FM modulation frequency in Hz
        """
        with self._thread_lock:
            if enable is not None:
                self._enable_fm = bool(enable)
            if deviation_khz is not None:
                self._fm_deviation_khz = float(deviation_khz)
            if modulation_frequency is not None:
                self._fm_modulation_frequency = float(modulation_frequency)

            self.log.info(f'FM parameters updated: enable={self._enable_fm}, '
                          f'deviation={self._fm_deviation_khz} kHz, '
                          f'mod_freq={self._fm_modulation_frequency} Hz')

            # If CW is currently on, update the configuration
            if self.module_state() != 'idle' and self._in_cw_mode:
                self.set_cw(self._current_rf_frequency, self._current_rf_power)

    def set_fm_per_component(self, component_index: int, fm_enabled: bool, fm_deviation_khz: float = None):
        """Set FM parameters for a specific component.

        @param int component_index: Index of the component (0-2)
        @param bool fm_enabled: Enable/disable FM for this component
        @param float fm_deviation_khz: FM deviation in kHz (optional)
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to change FM settings. Microwave output active.')

            if component_index >= len(self._if_frequencies):
                raise ValueError(f'Component index {component_index} out of range')

            # Store per-component FM settings
            if not hasattr(self, '_fm_enables_per_component'):
                self._fm_enables_per_component = [self._enable_fm] * len(self._if_frequencies)
            if not hasattr(self, '_fm_deviations_per_component'):
                self._fm_deviations_per_component = [self._fm_deviation_khz] * len(self._if_frequencies)

            self._fm_enables_per_component[component_index] = fm_enabled
            if fm_deviation_khz is not None:
                self._fm_deviations_per_component[component_index] = fm_deviation_khz

            self.log.info(f'Component {component_index} FM settings: enabled={fm_enabled}, '
                          f'deviation={self._fm_deviations_per_component[component_index]} kHz')

    def get_multi_frequency_info(self):
        """Get information about the current multi-frequency configuration.

        @return dict: Dictionary containing multi-frequency information
        """
        with self._thread_lock:
            avg_if = self._get_average_if_frequency()

            # Calculate actual RF frequencies for each IF component
            if self.module_state() != 'idle':
                try:
                    lo_freq = float(self._windfreak_device.query('f?')) * 1e6
                    rf_frequencies = [lo_freq - if_freq for if_freq in self._active_if_frequencies]
                except:
                    rf_frequencies = None
            else:
                rf_frequencies = None

            return {
                'mode': self._multi_frequency_mode,
                'active_if_frequencies': self._active_if_frequencies.copy(),
                'active_if_amplitudes': self._active_if_amplitudes.copy(),
                'average_if_frequency': avg_if,
                'rf_frequencies': rf_frequencies,
                'fm_enabled': self._enable_fm,
                'fm_deviation_khz': self._fm_deviation_khz,
                'fm_modulation_frequency': self._fm_modulation_frequency,
                'fm_enables_per_component': getattr(self, '_fm_enables_per_component',
                                                    [self._enable_fm] * len(self._if_frequencies)),
                'fm_deviations_per_component': getattr(self, '_fm_deviations_per_component',
                                                       [self._fm_deviation_khz] * len(self._if_frequencies))
            }