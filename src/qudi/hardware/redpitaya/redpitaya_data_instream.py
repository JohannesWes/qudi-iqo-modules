# -*- coding: utf-8 -*-
"""
Red Pitaya Data Input Streaming Module for Qudi.

This module provides continuous data streaming from the Red Pitaya using
pyrpl's scan module in stream mode. It implements the qudi DataInStreamInterface
for efficient continuous data acquisition at 125MHz/4096 ~ 30.5 kHz.

Architecture:
- Uses pyrpl scan module's stream mode
- Supports two input modes: demodulated lock-in data or FTW frequency corrections
- FPGA ring buffer (4096 samples) with continuous writing
- Software buffering for smooth data delivery
- Supports both CONTINUOUS and FINITE streaming modes

Example config:

    redpitaya_stream:
        module.Class: 'redpitaya.redpitaya_data_instream.RedPitayaDataInStream'
        options:
            redpitaya_config_name: 'rpy_shared_config'
            redpitaya_hostname: '192.168.1.100'
            channel_buffer_size: 100000    # Software buffer size (samples)
            calibration_factor: 1.0        # Multiply raw data by this factor
            signal_scale: 1.0              # Additional scaling factor
            max_fpga_read_samples: null    # Max samples per FPGA poll (null=read all)

            # Lock-in filter options (only used when stream_input='demod'):
            lock_in_fir_bypass_ch1: False  # True: CIC only (~15 kHz BW), False: CIC+FIR
            lock_in_fir_bypass_ch2: False
            lock_in_filter_ch1: '2kHz'     # Filter: '500Hz', '1kHz' (IIR), '1kHz_LP' (FIR LP),
                                           #         '1kHz_FIR' (FIR min-phase), '2kHz', '5kHz'
            lock_in_filter_ch2: '2kHz'

Usage Example:

    # In qudi logic module or console
    stream = qudi.get_hardware('redpitaya_stream')

    # Configure for continuous streaming
    stream.configure(
        active_channels=['ch1'],
        streaming_mode=StreamingMode.CONTINUOUS,
        channel_buffer_size=100000,
        sample_rate=30517
    )

    # Start stream
    stream.start_stream()

    # Read data in chunks
    for i in range(10):
        data, _ = stream.read_data(samples_per_channel=1000)
        print(f'Chunk {i}: {data.shape}, mean={data.mean():.3f}')
        time.sleep(0.1)

    # Stop stream
    stream.stop_stream()

Performance:
    - Fixed sample rate: ~30.517 kHz (125 MHz / 4096 FPGA decimation)
    - Typical latency: 5-20 ms per read
    - Software buffer prevents data loss under normal conditions
"""

import numpy as np
import time
from typing import Tuple, Union, Optional, List, Sequence

from qudi.core.configoption import ConfigOption
from qudi.util.mutex import RecursiveMutex
from qudi.util.constraints import ScalarConstraint
from qudi.interface.data_instream_interface import (
    DataInStreamInterface,
    DataInStreamConstraints,
    StreamingMode,
    SampleTiming
)
from .resource_manager import get_pyrpl_instance, release_pyrpl_instance


class RedPitayaDataInStream(DataInStreamInterface):
    """
    Red Pitaya continuous data streaming using pyrpl scan module.

    Provides high-speed continuous streaming of demodulated lock-in data
    at fixed ~30.517 kHz sample rate (125 MHz / 4096 decimation).

    The module uses pyrpl's scan module in stream mode, which writes samples
    continuously to a 4096-sample FPGA ring buffer. This module polls that
    buffer and maintains a larger software buffer for smooth data delivery.
    """

    # Config options
    _redpitaya_config_name = ConfigOption('redpitaya_config_name',
                                          default='rpy_shared_config', missing='info')
    _redpitaya_hostname = ConfigOption('redpitaya_hostname', missing='error')
    _calibration_factor = ConfigOption('calibration_factor', default=1.0, missing='info')
    _signal_scale = ConfigOption('signal_scale', default=1.0, missing='info')
    _default_buffer_size = ConfigOption('channel_buffer_size', default=100000, missing='info')
    _max_fpga_read_samples = ConfigOption('max_fpga_read_samples', default=None, missing='info')
    _stream_input = ConfigOption('stream_input', default='demod', missing='info')

    # Lock-in filter configuration (applies when stream_input='demod')
    # FIR bypass: True = CIC only (~15 kHz BW, ~160 µs latency), False = CIC+FIR
    _lock_in_fir_bypass_ch1 = ConfigOption('lock_in_fir_bypass_ch1', default=False, missing='info')
    _lock_in_fir_bypass_ch2 = ConfigOption('lock_in_fir_bypass_ch2', default=False, missing='info')
    # Filter selection (active when fir_bypass is False):
    # '500Hz', '1kHz' (IIR), '1kHz_LP' (FIR linear-phase), '1kHz_FIR' (FIR min-phase), '2kHz', '5kHz'
    _lock_in_filter_ch1 = ConfigOption('lock_in_filter_ch1', default='2kHz', missing='info')
    _lock_in_filter_ch2 = ConfigOption('lock_in_filter_ch2', default='2kHz', missing='info')

    # FPGA constants from pyrpl scan module
    _FPGA_CLOCK_FREQ = 125e6  # Hz
    _DEMOD_DECIMATION = 4096
    _STREAM_SAMPLE_RATE = _FPGA_CLOCK_FREQ / _DEMOD_DECIMATION  # ~30.517 kHz

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._pyrpl = None
        self._scan_module = None
        self._thread_lock = RecursiveMutex()

        # Constraints
        self._constraints = None

        # Current configuration
        self._active_channels = []
        self._streaming_mode = StreamingMode.INVALID
        self._channel_buffer_size = 0
        self._sample_rate = self._STREAM_SAMPLE_RATE
        self._current_stream_input = 'demod'  # Current input: 'demod' or 'ftw_corr'

        # Software buffer for smooth data delivery
        self._data_buffer = None  # numpy array
        self._buffer_write_pos = 0  # Write position in buffer
        self._buffer_read_pos = 0  # Read position in buffer
        self._total_samples_acquired = 0

        # Background polling
        self._poll_interval = 0.005  # 5ms between FPGA reads
        self._running = False

    def on_activate(self):
        """Initialize connection to Red Pitaya via pyrpl."""
        try:
            # Get shared pyrpl instance
            self._pyrpl, _ = get_pyrpl_instance(
                hostname=self._redpitaya_hostname,
                config_name=self._redpitaya_config_name
            )
            self.log.info(f'Acquired shared pyrpl instance for {self._redpitaya_hostname}')

            # Get scan module (will be used in stream mode)
            self._scan_module = self._pyrpl.rp.scan

            # Set initial stream input mode from config
            self._current_stream_input = self._stream_input
            self._scan_module.input_select = self._current_stream_input
            self.log.info(f'Stream input configured: {self._current_stream_input}')

            # Configure lock-in filter settings when using demod input
            if self._current_stream_input == 'demod':
                self._configure_lock_in_filters()

            # Create constraints
            self._constraints = DataInStreamConstraints(
                channel_units={'ch1': 'V'},  # Single channel (demodulated signal)
                sample_timing=SampleTiming.CONSTANT,  # Fixed rate
                streaming_modes=[StreamingMode.CONTINUOUS, StreamingMode.FINITE],
                data_type=np.float64,  # After calibration/scaling
                channel_buffer_size=ScalarConstraint(
                    default=self._default_buffer_size,
                    bounds=(1000, 10000000),  # 1k to 10M samples
                    increment=1
                ),
                sample_rate=ScalarConstraint(
                    default=self._STREAM_SAMPLE_RATE,
                    bounds=(self._STREAM_SAMPLE_RATE * 0.99,
                           self._STREAM_SAMPLE_RATE * 1.01),  # Narrow range (fixed)
                    increment=0
                )
            )

            # Initialize with default configuration
            self.configure(
                active_channels=['ch1'],
                streaming_mode=StreamingMode.CONTINUOUS,
                channel_buffer_size=self._default_buffer_size,
                sample_rate=self._STREAM_SAMPLE_RATE
            )

            self.log.info('Red Pitaya data instream module activated')
            self.log.info(f'Sample rate: {self._STREAM_SAMPLE_RATE:.1f} Hz (fixed)')

        except Exception as e:
            self.log.error(f'Failed to activate Red Pitaya instream: {e}')
            raise

    def on_deactivate(self):
        """Clean up resources."""
        try:
            # Stop streaming if active
            if self._running:
                self.stop_stream()

            # Release pyrpl instance
            if self._pyrpl is not None:
                release_pyrpl_instance(
                    hostname=self._redpitaya_hostname,
                    config_name=self._redpitaya_config_name
                )
                self._pyrpl = None

            self.log.info('Red Pitaya data instream module deactivated')

        except Exception as e:
            self.log.error(f'Error during deactivation: {e}')

    def _configure_lock_in_filters(self):
        """Configure lock-in module filter settings from config options.

        Applies FIR bypass and filter selection settings to the lock-in module.
        Called automatically during activation when stream_input='demod'.
        """
        try:
            # Module is named 'lockin' (not 'lock_in') per PyRPL naming convention
            lock_in = self._pyrpl.rp.lockin

            # Configure FIR bypass (True = CIC only, ~15 kHz BW, ~160 µs latency)
            lock_in.fir_bypass_ch1 = self._lock_in_fir_bypass_ch1
            lock_in.fir_bypass_ch2 = self._lock_in_fir_bypass_ch2

            # Configure filter selection (only active when FIR bypass is False)
            valid_filters = {'500Hz', '2kHz', '5kHz', '1kHz', '1kHz_LP', '1kHz_FIR'}

            if self._lock_in_filter_ch1 not in valid_filters:
                self.log.warning(f'Invalid lock_in_filter_ch1 "{self._lock_in_filter_ch1}". '
                                 f'Using "2kHz". Valid options: {valid_filters}')
                self._lock_in_filter_ch1 = '2kHz'
            lock_in.filter_select_ch1 = self._lock_in_filter_ch1

            if self._lock_in_filter_ch2 not in valid_filters:
                self.log.warning(f'Invalid lock_in_filter_ch2 "{self._lock_in_filter_ch2}". '
                                 f'Using "2kHz". Valid options: {valid_filters}')
                self._lock_in_filter_ch2 = '2kHz'
            lock_in.filter_select_ch2 = self._lock_in_filter_ch2

            # Log configuration summary
            ch1_mode = 'CIC only (~15 kHz)' if self._lock_in_fir_bypass_ch1 else f'CIC+FIR ({self._lock_in_filter_ch1})'
            ch2_mode = 'CIC only (~15 kHz)' if self._lock_in_fir_bypass_ch2 else f'CIC+FIR ({self._lock_in_filter_ch2})'
            self.log.info(f'Lock-in filters configured - Ch1: {ch1_mode}, Ch2: {ch2_mode}')

        except AttributeError as e:
            self.log.warning(f'Could not configure lock-in filters (module not available): {e}')
        except Exception as e:
            self.log.error(f'Failed to configure lock-in filters: {e}')

    @property
    def constraints(self) -> DataInStreamConstraints:
        """Read-only property returning the constraints on the settings for this data streamer."""
        return self._constraints

    @property
    def available_samples(self) -> int:
        """Number of samples available to read without blocking."""
        with self._thread_lock:
            if not self._running or self._data_buffer is None:
                return 0
            # Calculate available samples in circular buffer
            if self._buffer_write_pos >= self._buffer_read_pos:
                return self._buffer_write_pos - self._buffer_read_pos
            else:
                # Wrapped around
                return (self._channel_buffer_size - self._buffer_read_pos) + self._buffer_write_pos

    @property
    def sample_rate(self) -> float:
        """Read-only property returning the currently set sample rate in Hz."""
        return self._sample_rate

    @property
    def channel_buffer_size(self) -> int:
        """Read-only property returning the currently set buffer size in samples per channel."""
        return self._channel_buffer_size

    @property
    def streaming_mode(self) -> StreamingMode:
        """Read-only property returning the currently configured StreamingMode Enum."""
        return self._streaming_mode

    @property
    def active_channels(self) -> List[str]:
        """Read-only property returning the currently configured active channel names."""
        return self._active_channels.copy()

    @property
    def stream_input(self) -> str:
        """Read-only property returning current stream input mode ('demod' or 'ftw_corr')."""
        return self._current_stream_input

    def set_stream_input(self, input_mode: str) -> None:
        """
        Set streaming input source.

        Can be called while streaming is active or inactive. The FPGA register is
        always updated immediately to ensure the correct input is selected for
        subsequent operations (including ODMR scans that share the scan module).

        Args:
            input_mode: 'demod' for error signal or 'ftw_corr' for frequency correction

        Raises:
            ValueError: If invalid input_mode
        """
        with self._thread_lock:
            if input_mode not in ['demod', 'ftw_corr']:
                raise ValueError(f'Invalid input_mode: {input_mode}. Must be "demod" or "ftw_corr"')

            self._current_stream_input = input_mode

            # Always update FPGA register immediately, regardless of streaming state.
            # This is critical because other modules (e.g., RedPitayaFiniteSamplingInput
            # for ODMR scans) share the same physical scan module and need the correct
            # input_select setting.
            # Thread-safe: MonitorClient uses RLock to serialize TCP socket access
            if self._scan_module is not None:
                self._scan_module.input_select = input_mode
                if self._running:
                    self.log.info(f'Stream input switched live to: {input_mode}')
                else:
                    self.log.info(f'Stream input set to: {input_mode} (FPGA register updated)')

    def configure(self,
                  active_channels: Sequence[str],
                  streaming_mode: Union[StreamingMode, int],
                  channel_buffer_size: int,
                  sample_rate: float) -> None:
        """Configure a data stream. See read-only properties for information on each parameter."""
        with self._thread_lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Cannot configure data stream while it is already running')

            # Validate channels
            active_channels = list(active_channels)
            valid_channels = list(self._constraints.channel_units.keys())
            for ch in active_channels:
                if ch not in valid_channels:
                    raise ValueError(f'Invalid channel "{ch}". Valid channels: {valid_channels}')

            # Only single channel supported (demod output)
            if len(active_channels) > 1:
                self.log.warning('Only single channel supported. Using first channel.')
                active_channels = [active_channels[0]]

            # Validate streaming mode
            streaming_mode = StreamingMode(streaming_mode)
            if streaming_mode not in self._constraints.streaming_modes:
                raise ValueError(
                    f'Streaming mode {streaming_mode} not supported. '
                    f'Valid modes: {self._constraints.streaming_modes}'
                )

            # Validate buffer size
            if not self._constraints.channel_buffer_size.is_valid(channel_buffer_size):
                raise ValueError(
                    f'Buffer size {channel_buffer_size} out of bounds '
                    f'{self._constraints.channel_buffer_size.bounds}'
                )

            # Validate sample rate (must be close to FPGA rate)
            if not self._constraints.sample_rate.is_valid(sample_rate):
                self.log.warning(
                    f'Sample rate {sample_rate:.1f} Hz out of range. '
                    f'Using fixed FPGA rate {self._STREAM_SAMPLE_RATE:.1f} Hz'
                )
                sample_rate = self._STREAM_SAMPLE_RATE

            # Apply configuration
            self._active_channels = active_channels
            self._streaming_mode = streaming_mode
            self._channel_buffer_size = int(channel_buffer_size)
            self._sample_rate = float(sample_rate)

            self.log.debug(
                f'Configured: channels={self._active_channels}, '
                f'mode={self._streaming_mode.name}, '
                f'buffer={self._channel_buffer_size}, '
                f'rate={self._sample_rate:.1f} Hz'
            )

    def start_stream(self) -> None:
        """Start the data acquisition/streaming."""
        with self._thread_lock:
            assert self.module_state() == 'idle', \
                'Stream already running'
            assert self._streaming_mode != StreamingMode.INVALID, \
                'Must configure before starting stream'

            try:
                # Allocate software buffer (circular)
                self._data_buffer = np.zeros(self._channel_buffer_size, dtype=np.float64)
                self._buffer_write_pos = 0
                self._buffer_read_pos = 0
                self._total_samples_acquired = 0
        
                # Start FPGA streaming with configured input
                self._scan_module.stream_start(input_source=self._current_stream_input)

                # Mark as running
                self._running = True
                self.module_state.lock()

                self.log.info(f'Started {self._streaming_mode.name} stream (input: {self._current_stream_input})')

            except Exception as e:
                self._running = False
                if self.module_state() == 'locked':
                    self.module_state.unlock()
                raise RuntimeError(f'Failed to start stream: {e}')

    def stop_stream(self) -> None:
        """Stop the data acquisition/streaming."""
        with self._thread_lock:
            if not self._running:
                return

            try:
                # Stop FPGA streaming
                self._scan_module.stream_stop()

                # Mark as stopped
                self._running = False

                self.log.info(
                    f'Stopped stream. Total samples acquired: {self._total_samples_acquired}'
                )

            except Exception as e:
                self.log.error(f'Error stopping stream: {e}')
            finally:
                if self.module_state() == 'locked':
                    self.module_state.unlock()
                # Clear buffer
                self._data_buffer = None

    def _poll_fpga_and_update_buffer(self, max_samples_to_read=None):
        """
        Poll FPGA for new data and write to circular buffer.

        Args:
            max_samples_to_read: Maximum samples to request from FPGA per poll.
                                If None, uses config value (default: read all available).
                                Reading all available samples prevents FPGA ring buffer overflow.

        Returns:
            int: Number of new samples added to buffer
        """
        if not self._running:
            return 0

        # Use config value if not specified
        if max_samples_to_read is None:
            max_samples_to_read = self._max_fpga_read_samples

        try:
            # Read from FPGA stream (non-blocking)
            # If max_samples=None, stream_read() reads all available samples to prevent overflow
            # At ~30.5 kHz sample rate with 100ms polling, expect ~3000 samples per poll
            raw_data = self._scan_module.stream_read(max_samples=max_samples_to_read)

            if raw_data.size == 0:
                return 0

            # Apply calibration/conversion and scaling based on input mode
            if self._current_stream_input == 'ftw_corr':
                # FTW correction: convert to Hz using pyrpl's conversion
                calibrated_data = self._scan_module.ftw_to_hz(raw_data) * self._signal_scale
            else:
                # Demod: apply calibration factor
                calibrated_data = raw_data.astype(np.float64) * self._calibration_factor * self._signal_scale

            # Write to circular buffer
            n_samples = calibrated_data.size
            write_pos = self._buffer_write_pos
            buffer_size = self._channel_buffer_size

            # Check for buffer overflow (write catching up to read)
            space_available = buffer_size - self.available_samples - 1  # -1 to distinguish full/empty
            if n_samples > space_available:
                self.log.warning(
                    f'Software buffer overflow! Dropping {n_samples - space_available} samples. '
                    f'Consider increasing buffer size or reading faster.'
                )
                # In FINITE mode, this is critical
                if self._streaming_mode == StreamingMode.FINITE:
                    self.log.error('Buffer overflow in FINITE mode - data loss occurred!')

            # Write data (handle wraparound)
            if write_pos + n_samples <= buffer_size:
                # No wraparound
                self._data_buffer[write_pos:write_pos + n_samples] = calibrated_data
                self._buffer_write_pos = (write_pos + n_samples) % buffer_size
            else:
                # Wraparound
                first_chunk = buffer_size - write_pos
                self._data_buffer[write_pos:] = calibrated_data[:first_chunk]
                self._data_buffer[:n_samples - first_chunk] = calibrated_data[first_chunk:]
                self._buffer_write_pos = n_samples - first_chunk

            self._total_samples_acquired += n_samples

            # Check if FINITE mode target reached
            if self._streaming_mode == StreamingMode.FINITE:
                if self._total_samples_acquired >= self._channel_buffer_size:
                    self.log.debug('FINITE mode target reached, stopping stream')
                    self.stop_stream()

            return n_samples

        except Exception as e:
            self.log.error(f'Error polling FPGA: {e}')
            return 0

    def read_data_into_buffer(self,
                              data_buffer: np.ndarray,
                              samples_per_channel: int,
                              timestamp_buffer: Optional[np.ndarray] = None) -> None:
        """
        Read data from the stream buffer into a 1D numpy array given as parameter.

        Samples of all channels are stored interleaved in contiguous memory.
        In case of a multidimensional buffer array, this buffer will be flattened before written
        into.

        The data_buffer array must have the same data type as self.constraints.data_type.

        This function is blocking until the required number of samples has been acquired.
        """
        if timestamp_buffer is not None:
            raise NotImplementedError('Timestamp buffers not supported (SampleTiming.CONSTANT)')

        # Validate buffer
        data_buffer_flat = data_buffer.ravel()
        n_channels = len(self._active_channels)
        required_size = samples_per_channel * n_channels

        if data_buffer_flat.size < required_size:
            raise ValueError(
                f'Buffer too small. Need {required_size} elements, got {data_buffer_flat.size}'
            )

        if data_buffer.dtype != self._constraints.data_type:
            raise TypeError(
                f'Buffer dtype {data_buffer.dtype} does not match required {self._constraints.data_type}'
            )

        # Block until enough samples available
        timeout = max(30.0, samples_per_channel / self._sample_rate * 3)  # 3x expected time
        start_time = time.time()
        samples_read = 0

        while samples_read < samples_per_channel:
            # Poll FPGA for new data
            self._poll_fpga_and_update_buffer()

            # Read available samples
            available = self.available_samples
            if available > 0:
                to_read = min(available, samples_per_channel - samples_read)

                # Read from circular buffer
                read_pos = self._buffer_read_pos
                if read_pos + to_read <= self._channel_buffer_size:
                    # No wraparound
                    chunk = self._data_buffer[read_pos:read_pos + to_read]
                    self._buffer_read_pos = (read_pos + to_read) % self._channel_buffer_size
                else:
                    # Wraparound
                    first_chunk_size = self._channel_buffer_size - read_pos
                    chunk = np.concatenate([
                        self._data_buffer[read_pos:],
                        self._data_buffer[:to_read - first_chunk_size]
                    ])
                    self._buffer_read_pos = to_read - first_chunk_size

                # Write to output buffer (interleaved, but we only have 1 channel)
                data_buffer_flat[samples_read:samples_read + to_read] = chunk
                samples_read += to_read

            # Check timeout
            if time.time() - start_time > timeout:
                raise TimeoutError(
                    f'Timeout waiting for {samples_per_channel} samples. '
                    f'Only {samples_read} samples acquired after {timeout:.1f}s'
                )

            # Small sleep if no data available
            if available == 0:
                time.sleep(self._poll_interval)

    def read_available_data_into_buffer(self,
                                        data_buffer: np.ndarray,
                                        timestamp_buffer: Optional[np.ndarray] = None) -> int:
        """
        Read data from the stream buffer into a 1D numpy array given as parameter.

        All samples for each channel are stored in consecutive blocks one after the other.
        The number of samples read per channel is returned and can be used to slice out valid data
        from the buffer arrays.

        This method will read all currently available samples into buffer. If number of available
        samples exceeds buffer size, read only as many samples as fit into the buffer.
        """
        if timestamp_buffer is not None:
            raise NotImplementedError('Timestamp buffers not supported (SampleTiming.CONSTANT)')

        # Poll FPGA for latest data
        self._poll_fpga_and_update_buffer()

        # Validate buffer
        data_buffer_flat = data_buffer.ravel()
        n_channels = len(self._active_channels)

        if data_buffer.dtype != self._constraints.data_type:
            raise TypeError(
                f'Buffer dtype {data_buffer.dtype} does not match required {self._constraints.data_type}'
            )

        # Determine how many samples to read
        available = self.available_samples
        max_samples = data_buffer_flat.size // n_channels
        samples_to_read = min(available, max_samples)

        if samples_to_read == 0:
            return 0

        # Read from circular buffer (same logic as read_data_into_buffer)
        with self._thread_lock:
            read_pos = self._buffer_read_pos
            if read_pos + samples_to_read <= self._channel_buffer_size:
                chunk = self._data_buffer[read_pos:read_pos + samples_to_read]
                self._buffer_read_pos = (read_pos + samples_to_read) % self._channel_buffer_size
            else:
                first_chunk_size = self._channel_buffer_size - read_pos
                chunk = np.concatenate([
                    self._data_buffer[read_pos:],
                    self._data_buffer[:samples_to_read - first_chunk_size]
                ])
                self._buffer_read_pos = samples_to_read - first_chunk_size

        # Write to output buffer
        data_buffer_flat[:samples_to_read] = chunk

        return samples_to_read

    def read_data(self,
                  samples_per_channel: Optional[int] = None
                  ) -> Tuple[np.ndarray, Union[np.ndarray, None]]:
        """
        Read data from the stream buffer into a 1D numpy array and return it.

        All samples for each channel are stored in consecutive blocks one after the other.

        The numpy array data type is the one defined in self.constraints.data_type.

        If samples_per_channel is omitted all currently available samples are read from buffer.
        This method will not return until all requested samples have been read or a timeout occurs.
        """
        if samples_per_channel is None:
            # Non-blocking: read all available
            self._poll_fpga_and_update_buffer()
            samples_per_channel = self.available_samples
            if samples_per_channel == 0:
                return np.array([], dtype=self._constraints.data_type), None

        # Allocate buffer and read
        n_channels = len(self._active_channels)
        data_buffer = np.zeros(samples_per_channel * n_channels, dtype=self._constraints.data_type)
        self.read_data_into_buffer(data_buffer, samples_per_channel)

        # Reshape to (samples, channels)
        if n_channels > 1:
            data_buffer = data_buffer.reshape((samples_per_channel, n_channels))

        return data_buffer, None

    def read_single_point(self) -> Tuple[np.ndarray, Union[None, np.float64]]:
        """
        This method will initiate a single sample read on each configured data channel.

        In general this sample may not be acquired simultaneous for all channels and timing in
        general can not be assured. Use this method if you want to have a non-timing-critical
        snapshot of your current data channel input.

        The returned 1D numpy array will contain one sample for each channel.
        """
        data, _ = self.read_data(samples_per_channel=1)
        return data.ravel(), None  # Return 1D array with one value per channel
