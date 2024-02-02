# -*- coding: utf-8 -*-

"""
Modified SynthNV pro hardware file, for use with Max and Shravan's setup, without qudi.
"""
import time
import pyvisa
import numpy as np


class MicrowaveSynthNVPro:
    # @Max ToDO: Check which COM-port your windreak has and enter it here
    _serial_port = 'COM9'
    _comm_timeout = 10
    _output_channel = 0

    def __init__(self, *args, **kwargs):

        self._rm = None
        self._device = None
        self._model = ''
        self._constraints = None
        self._scan_power = -20
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
        self._scan_power = -20
        self._scan_frequencies = None
        self._in_cw_mode = True

    def on_deactivate(self):
        """ Cleanup performed during deactivation of the module.
        """
        self.off()
        self._device.close()
        self._device = None
        self._rm.close()
        self._rm = None

    @property
    def cw_power(self):
        """The CW microwave power in dBm. Must implement setter as well.

        @return float: The currently set CW microwave power in dBm.
        """
        return float(self._device.query('W?'))

    @property
    def cw_frequency(self):
        """The CW microwave frequency in Hz. Must implement setter as well.

         @return float: The currently set CW microwave frequency in Hz.
         """
        return float(self._device.query('f?')) * 1e6

    @property
    def scan_power(self):
        """The microwave power in dBm used for scanning. Must implement setter as well.

        @return float: The currently set scanning microwave power in dBm
        """
        return self._scan_power

    @property
    def scan_frequencies(self):
        """The microwave frequencies used for scanning. Must implement setter as well.

        This will be a tuple containing 3 values (freq_begin, freq_end, number_of_samples).
        If no frequency scan has been specified, return None.

        @return float[]: The currently set scanning frequencies. None if not set.
        """
        return self._scan_frequencies

    def set_cw(self, frequency, power):
        """Configure the CW microwave output. Does not start physical signal output, see also
        "cw_on".

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm
        """
        # self._assert_cw_parameters_args(frequency, power)

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

    def configure_scan(self, power, frequencies, sample_rate):
        """Configures power. Does not start physical signal output, see also
        "start_scan".

        @param float power: power to set in dBm
        @param tuple frequencies: (start, stop, points), with start and stop being the start and stop frequencies in Hz
                                    and points the number of frequency points in the list that will be generated
        @param float sample_rate: sample rate, that is 1/T where T is the period of one whole cycle of the trigger signal
        """

        # configure scan according to scan mode
        self._scan_power = power

        # configure step time ("dead time") of microwave -> for 50/50 duty cycle dead time ends at 0.75 of the whole
        # cycle, that is at a time when the trigger signal is high (MW doesn't change frequency then)
        self._device.write(f't{1000 * (0.75 / sample_rate):f}')
        self._scan_step_time = 0.75 / sample_rate
        self._scan_sample_rate = float(self._device.query('t?')) / 1000

        # necessary due to a bug with the temperature compensation in the current windfreak firmware
        # recommended by David Goins (Windfreak developer & owner)
        self._device.write('Z0')

        self._scan_frequencies = tuple(frequencies)
        self._write_sweep()

        # TESTING: Vielleicht macht es hier Sinn, etwas zu warten, um sicherzugehen, dass configuration done ist
        time.sleep(0.2)

    def off(self):
        """Switches off any microwave output (both scan and CW).
        Must return AFTER the device has actually stopped.
        """
        # disable sweep mode
        self._device.write('g0')
        # set trigger source to software
        self._device.write('y0')
        self._device.write('E0h0')

    def cw_on(self):
        """ Switches on cw microwave output.

        Must return AFTER the output is actually active.
        """
        self._in_cw_mode = True
        # enable sweep mode and set to start frequency
        self._device.write('g1')

    def start_scan(self):
        """Switches on the microwave scanning.

        Must return AFTER the output is actually active (and can receive triggers for example).
        """
        self._in_cw_mode = False

        # enable sweep mode and set to start frequency
        self._device.write('g1g0')

    def reset_scan(self):
        """Reset currently running scan and return to start frequency.
        """

        # enable sweep mode and set to start frequency
        self._device.write('g1g0')

    def _write_sweep(self):
        start, stop, points = self._scan_frequencies
        step = (stop - start) / (points - 1)

        # sweep mode: linear sweep, non-continuous (not automatically starting again, has to be reset via reset_scan())
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

    def _off(self):
        """ Turn the current channel off.

        @return tuple: see _stat()
        """
        self._device.write('E0h0')
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
