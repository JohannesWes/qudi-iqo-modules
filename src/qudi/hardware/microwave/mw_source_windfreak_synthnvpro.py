# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware module for the Windfreak SynthNVPro microwave source.
Contrary to the SynthHDPro, the SynthNVPro does not support the scan_mode JUMP_LIST.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
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
"""
import time

import pyvisa
import numpy as np

from qudi.util.mutex import Mutex
from qudi.core.configoption import ConfigOption
from qudi.interface.microwave_interface import MicrowaveInterface, MicrowaveConstraints
from qudi.util.enums import SamplingOutputMode


class MicrowaveSynthNVPro(MicrowaveInterface):
    """ Hardware class to control a SynthNV Pro only for mode EQUIDISTANT_SWEEP.

    Example config for copy-paste:

    mw_source_synthnv:
        module.Class: 'microwave.mw_source_windfreak_synthnvpro.MicrowaveSynthNVPro'
        options:
            serial_port: 'COM3'
            comm_timeout: 10  # in seconds
            output_channel: 0  # either 0 or 1
    """

    _serial_port = ConfigOption('serial_port', missing='error')
    _comm_timeout = ConfigOption('comm_timeout', default=10, missing='warn')
    _output_channel = ConfigOption('output_channel', 0, missing='info')


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # fixme: das bool(configoption) funktioniert hier glaube ich nicht, ich bekomme immer true.
        self._lock_in_FM_with_windfreak = bool(ConfigOption(name='lock_in_FM_with_windfreak', default=False))
        self._lock_in_FM_with_windfreak = False
        self._thread_lock = Mutex()
        self._rm = None
        self._device = None
        self._model = ''
        self._constraints = None
        self._scan_power = -20
        self._scan_mode = None
        self._modulation_frequency = 5000  # in Hz
        self._modulation_amplitude = 700000  # in Hz
        self._scan_frequencies = None
        self._scan_sample_rate = 0.
        self._scan_step_time = 0.
        self._in_cw_mode = True

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        # trying to load the visa connection to the module
        self._rm = pyvisa.ResourceManager()
        self._device = self._rm.open_resource(self._serial_port,
                                              baud_rate=9600,
                                              read_termination='\n',
                                              write_termination='\n',
                                              timeout=int(self._comm_timeout * 1000))
        self._model = self._device.query('+')

        ch = 0
        self.log.debug(f'Selected channel is Ch{ch}')

        # Generate constraints
        self._constraints = MicrowaveConstraints(
            power_limits=(-50, 13),
            frequency_limits=(12.5e6, 6.4e9),
            scan_size_limits=(2, 10000),
            sample_rate_limits=(0.1, 2500),
            scan_modes=(SamplingOutputMode.EQUIDISTANT_SWEEP, SamplingOutputMode.JUMP_LIST)
        )

        self._scan_power = -20
        self._scan_frequencies = None
        self._scan_sample_rate = self._constraints.max_sample_rate
        self._in_cw_mode = True
        print(f"activate, line 98: {self._lock_in_FM_with_windfreak}")

    def on_deactivate(self):
        """ Cleanup performed during deactivation of the module.
        """
        self.off()
        self._device.close()
        self._device = None
        self._rm.close()
        self._rm = None

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
            return float(self._device.query('W?'))

    @property
    def cw_frequency(self):
        """The CW microwave frequency in Hz. Must implement setter as well.

        @return float: The currently set CW microwave frequency in Hz.
        """
        with self._thread_lock:
            return float(self._device.query('f?')) * 1e6

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
            return SamplingOutputMode.EQUIDISTANT_SWEEP

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

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to set CW parameters. Microwave output active.')
            self._assert_cw_parameters_args(frequency, power)

            self._device.write('X0')
            self._device.write('c1')
            # trigger mode: software
            self._device.write('y0')
            self._device.write(f'W{power:2.3f}')
            self._device.write(f'[{power:2.3f}')
            self._device.write(f']{power:2.3f}')
            self._device.write(f'f{frequency / 1e6:5.7f}')
            self._device.write(f'l{frequency / 1e6:5.7f}')
            self._device.write(f'u{frequency / 1e6:5.7f}')

    def configure_scan(self, power, frequencies, mode, sample_rate):
        """
        """
        with self._thread_lock:
            # Sanity checks
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to configure frequency scan. Microwave output active.')
            self._assert_scan_configuration_args(power, frequencies, mode, sample_rate)

            # configure scan according to scan mode
            self._scan_power = power
            self._scan_mode = mode

            # either no lock-in is used, that is we have a normal frequency sweep
            # or the OPX is used for FM, which is controlled outside qudi
            if not self._lock_in_FM_with_windfreak:
                print("Not doing FM")
                # set step time, s.t. step time ("dead time") lies in part of trigger cycle where trigger == high, that
                # means the MW does not react (as it's low active)
                self._device.write(f't{1000 * 0.75 / sample_rate:f}')

            # the windfreak is used for FM
            else:
                # step time is reduced by one modulation step ...
                T_mod_cycle = 1 / self._modulation_frequency
                self._device.write(f't{1000 * (0.75 / sample_rate - T_mod_cycle):f}')
            self._scan_step_time = 0.75 / sample_rate
            self._scan_sample_rate = float(self._device.query('t?')) / 1000

            # necessary due to a bug with the temperature compensation in the current windfreak firmare
            # recommended by David Goins (Windfreak developer & owner)
            self._device.write('Z0')

            if self._lock_in_FM_with_windfreak:
                print(f"LOCK IN ON here self._lock_in true")
                assert mode == SamplingOutputMode.EQUIDISTANT_SWEEP, \
                    "Lock-In selected but mode != EQUIDISTANT_SWEEP"

                self._device.write(f'<{self._modulation_frequency}')
                self._device.write(f'>{self._modulation_amplitude}')
                # in my understanding, the WF completes 1 FM cycle and then checks if t > t_step. If so, and if also
                # trigger == LOW, it jumps to the next frequency point in the sweep, otherwise it starts the next
                # FM cycle at the current frequency point
                self._device.write(',1')

                # sets FM mode to sinuisoidal modulation (in the WF documentation 1 <-> 0  are swapped/wrong)
                self._device.write(';0')

                self._scan_frequencies = tuple(frequencies)
                self._write_sweep()


            elif mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                self._scan_frequencies = tuple(frequencies)
                self._write_sweep()

            elif mode == SamplingOutputMode.JUMP_LIST:
                print("CAREFUL: JJump list activated")
                self._scan_frequencies = np.asarray(frequencies, dtype=np.float64)
                self._write_list()

            # TESTING: Vielleicht macht es hier Sinn, etwas zu warten, um sicherzugehen, dass configuration done ist
            time.sleep(0.2)

            self.log.debug(f'Configured scan with: '
                           f'scan_power = {self._scan_power}, '
                           f'len(scan_frequencies) = {len(self._scan_frequencies)}, '
                           f'scan_sample_rate = {self._scan_sample_rate}')

    def off(self):
        """Switches off any microwave output (both scan and CW).
        Must return AFTER the device has actually stopped.
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                # # disable sweep mode
                # self._device.write('g0')
                # # set trigger source to software
                # self._device.write('y0')
                # # ToDo: Hier noch E0h0 schreiben, damit das device auch tatsächlich aus geht?
                # self._device.write('E0h0')
                # turn off everything for the current channel
                self.log.debug(f'Off: {self._off()}')
                self.module_state.unlock()

    def cw_on(self):
        """ Switches on cw microwave output.

        Must return AFTER the output is actually active.
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if self._in_cw_mode:
                    return
                raise RuntimeError(
                    'Unable to start CW microwave output. Microwave output is currently active.'
                )

            self._in_cw_mode = True
            self.log.debug(f'On: {self._on()}')
            # enable sweep mode and set to start frequency
            self._device.write('g1')
            self.module_state.lock()

    def start_scan(self):
        """Switches on the microwave scanning.

        Must return AFTER the output is actually active (and can receive triggers for example).
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if not self._in_cw_mode:
                    return
                raise RuntimeError('Unable to start frequency scan. CW microwave output is active.')
            assert self._scan_frequencies is not None, \
                'No scan_frequencies set. Unable to start scan.'

            self._in_cw_mode = False
            self.log.debug(f'start_scan: {self._on()}')
            # enable sweep mode and set to start frequency
            if self._scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                print("Scan Mode: Equidistant Sweep")
                if self._lock_in_FM_with_windfreak:
                    # starts FM
                    print("Using Lock-In")
                    self._device.write('/1')
                    # fixme: wäre nur g0 besser? springt bei g1 schon irgendwas los?
                self._device.write('g1g0')
            # nothing to be done for list mode
            else:
                pass

            self.module_state.lock()

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
                # enable sweep mode and set to start frequency
                self._device.write('g1g0')
            else:
                # TESTING: waiting before the reset, because otherwise the MW doesnt seem to spend time on last point
                time.sleep(self._scan_step_time)
                self._device.write('X1')
                self._device.write('g0')

    def _write_sweep(self):
        start, stop, points = self._scan_frequencies
        step = (stop - start) / (points - 1)

        # sweep mode: linear sweep, non-continuous
        self._device.write('X0')
        self._device.write('c0')

        # trigger mode: single step
        self._device.write('y2')

        # sweep direction
        if stop >= start:
            self._device.write('^1')
        else:
            self._device.write('^0')

        # sweep lower and upper frequency and steps
        self._device.write(f'l{start / 1e6:5.7f}')
        self._device.write(f'u{stop / 1e6:5.7f}')
        self._device.write(f's{step / 1e6:5.7f}')

        # set power
        self._device.write(f'W{self._scan_power:2.3f}')
        # set sweep lower end power
        self._device.write(f'[{self._scan_power:2.3f}')
        # set sweep upper end power
        self._device.write(f']{self._scan_power:2.3f}')

    def _write_list(self):
        # todo: implement

        # sweep mode: tabular sweep, non-continuous
        self._device.write('c0')
        self._device.write('X1')

        # trigger mode: single step
        self._device.write('y2')

        # delete possible old list
        self._device.write('Ld')

        # todo: create string from frequencies array and self._scan_power
        list_string = [f"L{ii}f{freq:.6f}L{ii}a-10.0" for ii, freq in enumerate(self._scan_frequencies / 1e6)]
        # write strings to device in two parts due to pyvisa chunk length of 20 kB
        list_string1, list_string2, list_string3 = "".join(list_string[0:175]), "".join(list_string[175:350]), "".join(
            list_string[350:])
        print(len(list_string1))
        print(len(list_string2))
        self._device.write(list_string1)
        # print(len(list_string2))
        time.sleep(0.1)
        if list_string2:
            self._device.write(list_string2)
        time.sleep(0.1)
        if list_string3:
            self._device.write(list_string3)

        # setzt g0 die WF im list Modus auch wieder auf den Anfang zurück?
        self._device.write('X1')
        self._device.write('g0')

        # sweep direction
        # todo: is it necessary to set the direction for the tabular mode?

        pass

    def _off(self):
        """ Turn the current channel off.

        @return tuple: see _stat()
        """
        #self._device.write('E0h0')
        return self._stat()

    def _on(self):
        """ Turn on the current channel.

        @return tuple(bool): see _stat()
        """
        self._device.write('W-20')
        self._device.write('E1h1')
        return self._stat()

    def _stat(self):
        """ Return status of PLL, power amplifier and output power muting for current channel.

        @return tuple(bool): PLL on, power amplifier on, output power muting on
        """
        # PLL status
        E = int(self._device.query('E?'))
        # hig/low power selector
        h = int(self._device.query('h?'))
        return E, h

    def _is_running(self):
        status = self._stat()
        return (status[0] == 1) and (status[1] == 1)
