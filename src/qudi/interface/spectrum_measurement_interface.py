# -*- coding: utf-8 -*-
"""
Interface for spectrum measurement devices (oscilloscopes with FFT, spectrum analyzers).
"""

from abc import abstractmethod
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np

from qudi.core.module import Base


@dataclass
class SpectrumMeasurementConstraints:
    """Constraints for spectrum measurement devices.

    Attributes:
        min_frequency_hz: Minimum measurable frequency in Hz
        max_frequency_hz: Maximum measurable frequency in Hz
        min_span_hz: Minimum frequency span in Hz
        max_span_hz: Maximum frequency span in Hz
        min_rbw_hz: Minimum resolution bandwidth in Hz
        max_rbw_hz: Maximum resolution bandwidth in Hz
        max_markers: Maximum number of markers available
    """
    min_frequency_hz: float = 0
    max_frequency_hz: float = 5e9
    min_span_hz: float = 100
    max_span_hz: float = 5e9
    min_rbw_hz: float = 1
    max_rbw_hz: float = 10e6
    max_markers: int = 8


class SpectrumMeasurementInterface(Base):
    """Interface for spectrum/FFT measurement devices.

    This interface provides methods for controlling devices capable of measuring
    power spectral density, such as:
    - Oscilloscopes with FFT capability
    - RF spectrum analyzers
    - Signal analyzers

    The interface supports:
    - Frequency domain configuration (center frequency, span, resolution bandwidth)
    - Marker-based point measurements
    - Full spectrum trace acquisition
    - Single and continuous acquisition modes
    """

    @property
    @abstractmethod
    def constraints(self) -> SpectrumMeasurementConstraints:
        """Return device constraints.

        Returns:
            SpectrumMeasurementConstraints: Hardware constraints object
        """
        pass

    @property
    @abstractmethod
    def center_frequency(self) -> float:
        """Get the current center frequency in Hz.

        Returns:
            float: Center frequency in Hz
        """
        pass

    @center_frequency.setter
    @abstractmethod
    def center_frequency(self, frequency_hz: float) -> None:
        """Set the center frequency for FFT/spectrum measurement.

        Args:
            frequency_hz: Center frequency in Hz
        """
        pass

    @property
    @abstractmethod
    def span(self) -> float:
        """Get the current frequency span in Hz.

        Returns:
            float: Frequency span in Hz
        """
        pass

    @span.setter
    @abstractmethod
    def span(self, span_hz: float) -> None:
        """Set the frequency span.

        Args:
            span_hz: Frequency span in Hz
        """
        pass

    @property
    @abstractmethod
    def resolution_bandwidth(self) -> float:
        """Get the current resolution bandwidth in Hz.

        Returns:
            float: Resolution bandwidth in Hz
        """
        pass

    @resolution_bandwidth.setter
    @abstractmethod
    def resolution_bandwidth(self, rbw_hz: float) -> None:
        """Set the resolution bandwidth.

        Args:
            rbw_hz: Resolution bandwidth in Hz
        """
        pass

    @abstractmethod
    def trigger_single(self) -> None:
        """Trigger a single acquisition.

        This method blocks until the acquisition is complete and data is ready.
        """
        pass

    @abstractmethod
    def start_continuous(self) -> None:
        """Start continuous acquisition mode.

        The device will continuously acquire spectra until stop_continuous() is called.
        """
        pass

    @abstractmethod
    def stop_continuous(self) -> None:
        """Stop continuous acquisition mode."""
        pass

    @abstractmethod
    def get_spectrum(self) -> Tuple[np.ndarray, np.ndarray]:
        """Acquire and return the current spectrum.

        This method triggers a single acquisition (if not in continuous mode)
        and returns the spectrum data.

        Returns:
            tuple: (frequency_array, amplitude_array)
                - frequency_array: 1D numpy array of frequencies in Hz
                - amplitude_array: 1D numpy array of amplitudes in dBm (or linear units)
        """
        pass

    @abstractmethod
    def set_marker_frequency(self, marker_id: int, frequency_hz: float) -> None:
        """Set the frequency position of a marker.

        Args:
            marker_id: Marker identifier (typically 1-based)
            frequency_hz: Target frequency in Hz
        """
        pass

    @abstractmethod
    def get_marker_amplitude(self, marker_id: int) -> float:
        """Query the amplitude at a marker position.

        The marker should be positioned at the desired frequency before calling
        this method using set_marker_frequency().

        Args:
            marker_id: Marker identifier (typically 1-based)

        Returns:
            float: Amplitude at marker position in dBm (or linear units)
        """
        pass

    @abstractmethod
    def enable_marker(self, marker_id: int, enable: bool = True) -> None:
        """Enable or disable a marker.

        Args:
            marker_id: Marker identifier (typically 1-based)
            enable: True to enable, False to disable
        """
        pass

    def get_marker_position(self, marker_id: int) -> Tuple[float, float]:
        """Query the frequency and amplitude at a marker position.

        Default implementation calls get_marker_frequency and get_marker_amplitude.
        Implementations may override for efficiency.

        Args:
            marker_id: Marker identifier (typically 1-based)

        Returns:
            tuple: (frequency_hz, amplitude_dbm)
        """
        freq = self.get_marker_frequency(marker_id)
        amp = self.get_marker_amplitude(marker_id)
        return freq, amp

    def get_marker_frequency(self, marker_id: int) -> float:
        """Query the current frequency of a marker.

        Default implementation raises NotImplementedError. Subclasses should
        override if the device supports reading marker positions.

        Args:
            marker_id: Marker identifier (typically 1-based)

        Returns:
            float: Current marker frequency in Hz
        """
        raise NotImplementedError("get_marker_frequency not implemented for this device")

    def measure_power_at_frequency(self, frequency_hz: float, marker_id: int = 1) -> float:
        """Convenience method to measure power at a specific frequency.

        This method:
        1. Sets the marker to the specified frequency
        2. Triggers a single acquisition
        3. Returns the marker amplitude

        Args:
            frequency_hz: Target frequency in Hz
            marker_id: Marker to use (default: 1)

        Returns:
            float: Power at the specified frequency in dBm
        """
        self.set_marker_frequency(marker_id, frequency_hz)
        self.trigger_single()
        return self.get_marker_amplitude(marker_id)
