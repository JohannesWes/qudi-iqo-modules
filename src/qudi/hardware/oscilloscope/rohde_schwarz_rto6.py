# -*- coding: utf-8 -*-
"""
Rohde & Schwarz RTO6 oscilloscope hardware module for spectrum measurements.

This module provides FFT-based spectrum analysis using the RTO6 oscilloscope's
math functions. The current implementation is primarily intended for IQ mixer calibration applications.
"""

from typing import Tuple, Optional
import numpy as np
from time import sleep

from qudi.core.module import Base
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.interface.spectrum_measurement_interface import (
    SpectrumMeasurementInterface,
    SpectrumMeasurementConstraints
)

try:
    from RsInstrument import RsInstrument
except ImportError:
    RsInstrument = None


class RhodeSchwarzRTO6(SpectrumMeasurementInterface):
    """Rohde & Schwarz RTO6 oscilloscope hardware module for spectrum measurements.

    This module uses the RTO6's FFT math function (CALC:MATH1) to perform spectrum
    analysis. It provides marker-based power measurements suitable for e.g. IQ mixer
    calibration workflows.

    Example configuration:
        hardware:
            rto6_spectrum:
                module.Class: 'oscilloscope.rohde_schwarz_rto6.RhodeSchwarzRTO6'
                options:
                    visa_address: 'TCPIP0::10.203.129.15::inst0::INSTR'
                    timeout_ms: 5000
                    input_channel: 1
                    input_coupling: 'DC'
    """

    # ConfigOptions
    _visa_address = ConfigOption(
        name='visa_address',
        missing='error'
    )
    _timeout_ms = ConfigOption(
        name='timeout_ms',
        default=5000
    )
    _input_channel = ConfigOption(
        name='input_channel',
        default=1
    )
    _input_coupling = ConfigOption(
        name='input_coupling',
        default='DC'
    )
    _time_scale = ConfigOption(
        name='time_scale',
        default=5e-9
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = Mutex()
        self._device: Optional[RsInstrument] = None
        self._constraints: Optional[SpectrumMeasurementConstraints] = None

        # Cached state
        self._center_frequency_hz: float = 0
        self._span_hz: float = 100e6
        self._rbw_hz: float = 10e3
        self._is_continuous: bool = False

    def on_activate(self) -> None:
        """Initialize connection to RTO6 and configure for FFT measurements."""
        if RsInstrument is None:
            raise ImportError(
                "RsInstrument library is required for RTO6 hardware. "
                "Install with: pip install RsInstrument"
            )

        self.log.info(f"Connecting to RTO6 at {self._visa_address}")

        self._device = RsInstrument(
            self._visa_address,
            id_query=True,
            reset=True,
            options="SelectVisa='rs'"
        )
        sleep(0.5)

        # Configure timeouts
        self._device.visa_timeout = self._timeout_ms
        self._device.opc_timeout = self._timeout_ms
        self._device.instrument_status_checking = True
        self._device.clear_status()

        # Get device identification
        idn = self._device.query_str('*IDN?')
        self.log.info(f'Connected to: {idn}')

        # Initial setup for FFT mode
        self._setup_fft_mode()

        # Build constraints based on device type
        self._constraints = SpectrumMeasurementConstraints(
            min_frequency_hz=0,
            max_frequency_hz=6e9,  # Typical RTO6 analog bandwidth
            min_span_hz=100,
            max_span_hz=6e9,
            min_rbw_hz=1,
            max_rbw_hz=10e6,
            max_markers=8
        )

        self.log.info("RTO6 initialized in FFT mode")

    def _setup_fft_mode(self) -> None:
        """Configure the oscilloscope for FFT measurements."""
        with self._thread_lock:
            # Enable display updates
            self._device.write_str_with_opc('SYST:DISP:UPD ON')

            # Configure Math1 for FFT magnitude of input channel
            channel = int(self._input_channel)
            self._device.write_str_with_opc(f'CALC:MATH1 "FFTmag(Ch{channel})"')
            self._device.write_str_with_opc('CALC:MATH1:STATE ON')

            # Set time scale
            self._device.write_str_with_opc(f'TIM:SCAL {self._time_scale}')

            # Set input coupling
            self._device.write_str_with_opc(f'CHAN{channel}:COUP {self._input_coupling}')

    def on_deactivate(self) -> None:
        """Clean up connection to RTO6."""
        if self._device is not None:
            try:
                self._device.clear_status()
                self._device.close()
                self.log.info("RTO6 connection closed")
            except Exception as e:
                self.log.warning(f"Error closing RTO6 connection: {e}")
            finally:
                self._device = None

    # ─────────────────────────────────────────────────────────────────────────
    # SpectrumMeasurementInterface Properties
    # ─────────────────────────────────────────────────────────────────────────

    @property
    def constraints(self) -> SpectrumMeasurementConstraints:
        """Return device constraints."""
        return self._constraints

    @property
    def center_frequency(self) -> float:
        """Get the current center frequency in Hz."""
        return self._center_frequency_hz

    @center_frequency.setter
    def center_frequency(self, frequency_hz: float) -> None:
        """Set the center frequency for FFT measurement."""
        with self._thread_lock:
            self._device.write_str_with_opc(f'CALC:MATH1:FFT:CFR {int(frequency_hz)}')
            self._center_frequency_hz = frequency_hz
            self.log.debug(f"Set center frequency to {frequency_hz / 1e6:.3f} MHz")

    @property
    def span(self) -> float:
        """Get the current frequency span in Hz."""
        return self._span_hz

    @span.setter
    def span(self, span_hz: float) -> None:
        """Set the frequency span."""
        with self._thread_lock:
            self._device.write_str_with_opc(f'CALC:MATH1:FFT:SPAN {int(span_hz)}')
            self._span_hz = span_hz
            self.log.debug(f"Set span to {span_hz / 1e6:.1f} MHz")

    @property
    def resolution_bandwidth(self) -> float:
        """Get the current resolution bandwidth in Hz."""
        return self._rbw_hz

    @resolution_bandwidth.setter
    def resolution_bandwidth(self, rbw_hz: float) -> None:
        """Set the resolution bandwidth."""
        with self._thread_lock:
            self._device.write_str_with_opc(f'CALC:MATH1:FFT:BAND {int(rbw_hz)}')
            self._rbw_hz = rbw_hz
            self.log.debug(f"Set RBW to {rbw_hz / 1e3:.1f} kHz")

    # ─────────────────────────────────────────────────────────────────────────
    # SpectrumMeasurementInterface Methods
    # ─────────────────────────────────────────────────────────────────────────

    def trigger_single(self) -> None:
        """Trigger a single acquisition."""
        with self._thread_lock:
            self._device.write_str_with_opc('SING')

    def start_continuous(self) -> None:
        """Start continuous acquisition mode."""
        with self._thread_lock:
            self._device.write_str_with_opc('RUN')
            self._is_continuous = True
            self.log.debug("Started continuous acquisition")

    def stop_continuous(self) -> None:
        """Stop continuous acquisition mode."""
        with self._thread_lock:
            self._device.write_str_with_opc('STOP')
            self._is_continuous = False
            self.log.debug("Stopped continuous acquisition")

    def get_spectrum(self) -> Tuple[np.ndarray, np.ndarray]:
        """Acquire and return the current spectrum.

        Returns:
            tuple: (frequency_array, amplitude_array) where frequencies are in Hz
                   and amplitudes are in dB (relative to full scale)
        """
        with self._thread_lock:
            # Trigger acquisition if not in continuous mode
            if not self._is_continuous:
                self._device.write_str_with_opc('SING')

            # Get FFT data
            amp_data = self._device.query_bin_or_ascii_float_list(
                'FORM REAL,32;CALC:MATH1:DATA?'
            )

            # Get header info (start freq, stop freq, num points)
            header = self._device.query_bin_or_ascii_float_list('CALC:MATH1:DATA:HEAD?')
            start_freq = header[0]
            stop_freq = header[1]
            num_points = int(header[2])

            # Generate frequency vector
            freq_array = np.linspace(start_freq, stop_freq, num_points)
            amp_array = np.array(amp_data)

            self.log.debug(
                f"Retrieved spectrum: {num_points} points, "
                f"{start_freq / 1e6:.3f} - {stop_freq / 1e6:.3f} MHz"
            )

            return freq_array, amp_array

    def enable_marker(self, marker_id: int, enable: bool = True) -> None:
        """Enable or disable a marker.

        Args:
            marker_id: Marker identifier (1-8)
            enable: True to enable, False to disable
        """
        with self._thread_lock:
            state = 'ON' if enable else 'OFF'
            self._device.write_str_with_opc(f'CURS{int(marker_id)}:STAT {state}')

            if enable:
                # Configure marker to track Math1 (FFT) trace
                self._device.write_str_with_opc(f'CURS{int(marker_id)}:SOUR M1')
                self._device.write_str_with_opc(f'CURS{int(marker_id)}:TRAC ON')

            self.log.debug(f"Marker {marker_id} {'enabled' if enable else 'disabled'}")

    def set_marker_frequency(self, marker_id: int, frequency_hz: float) -> None:
        """Set the frequency position of a marker.

        Args:
            marker_id: Marker identifier (1-8)
            frequency_hz: Target frequency in Hz
        """
        with self._thread_lock:
            # Trigger acquisition to ensure valid FFT data before positioning marker
            # (matches legacy behavior from auto_mixer_tools_visa.py)
            self._device.write_str_with_opc('SING')

            # Ensure marker is enabled and tracking FFT
            self._device.write_str_with_opc(f'CURS{int(marker_id)}:STAT ON')
            self._device.write_str_with_opc(f'CURS{int(marker_id)}:SOUR M1')
            self._device.write_str_with_opc(f'CURS{int(marker_id)}:TRAC ON')

            # Set X position (frequency)
            self._device.write(f'CURS{int(marker_id)}:X1P {int(frequency_hz)}')

            self.log.debug(f"Set marker {marker_id} to {frequency_hz / 1e6:.3f} MHz")

    def get_marker_amplitude(self, marker_id: int) -> float:
        """Query the amplitude at a marker position.

        Args:
            marker_id: Marker identifier (1-8)

        Returns:
            float: Amplitude at marker position in dBm
        """
        with self._thread_lock:
            value = self._device.query_float(f'CURS{int(marker_id)}:Y1P?')
            self.log.debug(f"Marker {marker_id} amplitude: {value:.2f} dB")
            return float(value)

    def get_marker_frequency(self, marker_id: int) -> float:
        """Query the current frequency of a marker.

        Args:
            marker_id: Marker identifier (1-8)

        Returns:
            float: Current marker frequency in Hz
        """
        with self._thread_lock:
            value = self._device.query_float(f'CURS{int(marker_id)}:X1P?')
            return float(value)

    # ─────────────────────────────────────────────────────────────────────────
    # Additional RTO6-Specific Methods
    # ─────────────────────────────────────────────────────────────────────────

    def configure_for_calibration(
            self,
            center_freq_hz: float,
            span_hz: float,
            lo_marker_freq_hz: float,
            image_marker_freq_hz: float
    ) -> None:
        """Configure the oscilloscope for IQ calibration measurements.

        This convenience method sets up:
        - Center frequency and span
        - Marker 1 at LO frequency (for leakage measurement)
        - Marker 2 at image frequency (for image rejection measurement)

        Args:
            center_freq_hz: Center frequency for FFT display
            span_hz: Frequency span
            lo_marker_freq_hz: LO leakage frequency (marker 1)
            image_marker_freq_hz: Image frequency (marker 2)
        """
        self.center_frequency = center_freq_hz
        self.span = span_hz

        # Configure markers
        self.set_marker_frequency(1, lo_marker_freq_hz)
        self.set_marker_frequency(2, image_marker_freq_hz)

        self.log.info(
            f"Configured for calibration: center={center_freq_hz / 1e6:.3f} MHz, "
            f"span={span_hz / 1e6:.1f} MHz, "
            f"LO marker={lo_marker_freq_hz / 1e6:.3f} MHz, "
            f"image marker={image_marker_freq_hz / 1e6:.3f} MHz"
        )

    def measure_calibration_powers(self) -> Tuple[float, float]:
        """Measure LO leakage and image power in a single acquisition.

        Returns:
            tuple: (lo_leakage_dbm, image_power_dbm)
        """
        with self._thread_lock:
            self._device.write_str_with_opc('SING')
            lo_power = self._device.query_float('CURS1:Y1P?')
            image_power = self._device.query_float('CURS2:Y1P?')
            return float(lo_power), float(image_power)

    def set_input_range(self, range_v: float) -> None:
        """Set the input voltage range.

        Args:
            range_v: Full-scale input range in volts
        """
        with self._thread_lock:
            channel = int(self._input_channel)
            self._device.write_str_with_opc(f'CHAN{channel}:RANG {range_v}')
            self.log.debug(f"Set input range to {range_v} V")

    def set_input_offset(self, offset_v: float) -> None:
        """Set the input offset.

        Args:
            offset_v: DC offset in volts
        """
        with self._thread_lock:
            channel = int(self._input_channel)
            self._device.write_str_with_opc(f'CHAN{channel}:OFFS {offset_v}')
            self.log.debug(f"Set input offset to {offset_v} V")

    def query_instrument_id(self) -> str:
        """Query the instrument identification string.

        Returns:
            str: Instrument identification (*IDN? response)
        """
        with self._thread_lock:
            return self._device.query_str('*IDN?')
