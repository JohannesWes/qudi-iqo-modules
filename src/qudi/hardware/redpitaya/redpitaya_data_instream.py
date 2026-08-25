# -*- coding: utf-8 -*-
"""
Red Pitaya Data Input Streaming Module for Qudi.

This module provides continuous data streaming from the Red Pitaya using
pyrpl's scan module in *push* stream mode. It implements the qudi
DataInStreamInterface for efficient continuous data acquisition at
125MHz/4096 ~ 30.5 kHz.

Architecture:
- Uses pyrpl scan module's ARM-side-drain + TCP-push streaming
  (``scan.push_stream_start/read/stop``). The real-time deadline lives on the
  Red Pitaya ARM core, not on this PC, so GC pauses / Qt event-loop stalls /
  network jitter no longer cause data loss.
- A background receiver thread on the PC (pyrpl ``StreamClient``) drains the
  socket continuously into its own buffer; this module only pulls from it.
- Supports two input modes: demodulated lock-in data or FTW frequency corrections
- Lost samples (genuine FPGA-ring overruns only) are reported explicitly and
  **NaN-filled** so the time axis stays truthful -- never silently dropped.
- Supports both CONTINUOUS and FINITE streaming modes

See ``docs/developer_guide/scan_push_streaming.md`` in the pyrpl repo for the
full design and wire protocol.

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
            lock_in_filter_ch1: '2kHz_minphase'  # '2kHz_minphase' or '2kHz_linear'
            lock_in_filter_ch2: '2kHz_minphase'

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
    - ARM-side drain + TCP push keeps the real-time deadline off the PC; the
      receiver thread tolerates PC-side stalls (socket buffers ~1 s headroom)
    - No data loss under normal load; genuine overruns are NaN-filled and logged
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

    The module uses pyrpl's scan module in *push* stream mode: the Red Pitaya
    ARM core continuously drains the 4096-sample FPGA ring buffer and pushes
    framed samples over a dedicated TCP socket. A pyrpl ``StreamClient`` daemon
    thread receives them on the PC; this module pulls from that receiver on
    demand. Lost samples (genuine FPGA-ring overruns only) are NaN-filled rather
    than silently dropped, keeping the time axis truthful.
    """

    # Config options
    _redpitaya_config_name = ConfigOption('redpitaya_config_name',
                                          default='rpy_shared_config', missing='info')
    _redpitaya_hostname = ConfigOption('redpitaya_hostname', missing='error')
    _calibration_factor = ConfigOption('calibration_factor', default=1.0, missing='info')
    _signal_scale = ConfigOption('signal_scale', default=1.0, missing='info')
    _default_buffer_size = ConfigOption('channel_buffer_size', default=100000, missing='info')
    # Deprecated/unused with push streaming (the ARM server drains the FPGA ring
    # continuously, so there is no per-poll read cap). Kept for config back-compat.
    _max_fpga_read_samples = ConfigOption('max_fpga_read_samples', default=None, missing='info')
    _stream_input = ConfigOption('stream_input', default='demod', missing='info')

    # Push-streaming headroom tuning (passed to scan.push_stream_start):
    #   stream_ring_bytes : ARM-side DRAM ring size, sets how long a PC stall can
    #                       last before any sample is lost (ring_bytes/wire_rate
    #                       seconds). 0 = pyrpl default (16 MB ~ tens of seconds).
    #                       e.g. 67108864 (64 MB) for ~1-2 min of headroom.
    #   stream_coalesce_us: max ARM batching latency in microseconds (0 = 5 ms
    #                       default). Larger trims frame-header overhead.
    _stream_ring_bytes = ConfigOption('stream_ring_bytes', default=0, missing='nothing')
    _stream_coalesce_us = ConfigOption('stream_coalesce_us', default=0, missing='nothing')

    # Lock-in filter configuration (applies when stream_input='demod')
    # FIR bypass: True = CIC only (~15 kHz BW, ~160 µs latency), False = CIC+FIR
    _lock_in_fir_bypass_ch1 = ConfigOption('lock_in_fir_bypass_ch1', default=False, missing='info')
    _lock_in_fir_bypass_ch2 = ConfigOption('lock_in_fir_bypass_ch2', default=False, missing='info')
    # Filter selection (active when fir_bypass is False):
    # Both filters have the same CIC-compensated 2 kHz magnitude response.
    _lock_in_filter_ch1 = ConfigOption('lock_in_filter_ch1', default='2kHz_minphase', missing='info')
    _lock_in_filter_ch2 = ConfigOption('lock_in_filter_ch2', default='2kHz_minphase', missing='info')

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

        # KDC_HW_SYNC handover state: the board streams ONE quantity at a time, so a
        # marker-mode scan must take over this single stream. begin_scan_stream()
        # remembers the monitor stream here and end_scan_stream() restores it.
        self._scan_active = False
        self._scan_resume = False
        self._scan_resume_input = 'demod'
        self._scan_tap = None        # display tap into the scan's feed (fan-out)

        # Push-streaming receiver (pyrpl StreamClient) + a small FIFO of samples
        # already drained from it but not yet handed to the consumer. The
        # StreamClient's own daemon thread does the continuous buffering, so we
        # no longer maintain a circular buffer or a PC-side polling deadline.
        self._rx = None              # pyrpl StreamClient
        self._pending = np.empty(0, dtype=np.float64)  # calibrated, unconsumed
        self._total_samples_acquired = 0
        self._last_gap_reported = 0  # for gap (NaN) delta logging

        # Sleep interval while blocking for more samples in read_data_into_buffer
        self._poll_interval = 0.005
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
            valid_filters = {'2kHz_minphase', '2kHz_linear', '2kHz'}

            if self._lock_in_filter_ch1 not in valid_filters:
                self.log.warning(f'Invalid lock_in_filter_ch1 "{self._lock_in_filter_ch1}". '
                                 f'Using "2kHz_minphase". Valid options: {valid_filters}')
                self._lock_in_filter_ch1 = '2kHz_minphase'
            lock_in.filter_select_ch1 = self._lock_in_filter_ch1

            if self._lock_in_filter_ch2 not in valid_filters:
                self.log.warning(f'Invalid lock_in_filter_ch2 "{self._lock_in_filter_ch2}". '
                                 f'Using "2kHz_minphase". Valid options: {valid_filters}')
                self._lock_in_filter_ch2 = '2kHz_minphase'
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
        """Number of samples available to read without blocking.

        Counts samples already drained into the local FIFO plus those still
        buffered in the receiver thread (each is 1:1 on the stream timeline,
        including NaN-filled gap samples).
        """
        with self._thread_lock:
            n = int(self._pending.size)
            if self._running and self._rx is not None:
                n += int(self._rx.available())
            return n

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
        """Start the data acquisition/streaming.

        While a KDC_HW_SYNC scan owns the board's single stream, this does NOT
        start a competing stream (which would reset the shared counter and break
        the scan). Instead it attaches a read-only *tap* to the scan's live feed,
        so a time-trace can display the SAME samples the scan is binning.
        """
        with self._thread_lock:
            if self._scan_active:
                if self._rx is None or not getattr(self._rx, 'running', False):
                    self._scan_tap = self._scan_module.add_stream_tap()
                    self._rx = self._scan_tap
                self._pending = np.empty(0, dtype=np.float64)
                self._total_samples_acquired = 0
                self._last_gap_reported = 0
                self._running = True
                if self.module_state() != 'locked':
                    self.module_state.lock()
                self.log.info('Time-trace attached to the running scan stream (tap, '
                              f'input={self._current_stream_input}).')
                return

            assert self.module_state() == 'idle', \
                'Stream already running'
            assert self._streaming_mode != StreamingMode.INVALID, \
                'Must configure before starting stream'

            try:
                # Reset local FIFO and counters
                self._pending = np.empty(0, dtype=np.float64)
                self._total_samples_acquired = 0
                self._last_gap_reported = 0

                # Start push streaming: selects input, resets+enables the FPGA
                # stream engine, lazily deploys+starts the ARM server, and starts
                # the PC-side receiver thread. Returns the StreamClient. The ring
                # size sets the PC-stall headroom (see config options above).
                self._rx = self._scan_module.push_stream_start(
                    input_source=self._current_stream_input,
                    ring_bytes=int(self._stream_ring_bytes),
                    coalesce_us=int(self._stream_coalesce_us))

                # Mark as running
                self._running = True
                self.module_state.lock()

                self.log.info(f'Started {self._streaming_mode.name} push stream '
                              f'(input: {self._current_stream_input})')

            except Exception as e:
                self._running = False
                self._rx = None
                if self.module_state() == 'locked':
                    self.module_state.unlock()
                raise RuntimeError(f'Failed to start stream: {e}')

    def stop_stream(self) -> None:
        """Stop the data acquisition/streaming.

        Any samples still buffered in the receiver are drained into the local
        FIFO so a consumer can read the tail after stopping; the FIFO is cleared
        on the next ``start_stream``.

        While a scan owns the stream, this only detaches the display tap -- the
        scan's stream is left running untouched.
        """
        with self._thread_lock:
            if self._scan_active:
                if self._scan_tap is not None:
                    self._scan_module.remove_stream_tap(self._scan_tap)
                    self._scan_tap = None
                self._rx = None
                self._running = False
                if self.module_state() == 'locked':
                    self.module_state.unlock()
                self.log.info('Time-trace detached from scan stream (scan continues).')
                return
            self._do_stop()

    def _do_stop(self) -> None:
        """Internal stop: disable the FPGA stream engine, stop the receiver,
        and drain any remaining samples into the local FIFO. Must be called
        under ``self._thread_lock``. Used by both ``stop_stream`` and the
        FINITE-mode auto-stop."""
        if not self._running:
            return
        # Clear the running flag first so the FINITE auto-stop inside _drain_rx()
        # (reached via the drain below) short-circuits instead of re-entering.
        self._running = False
        try:
            # Stops the PC receiver thread AND disables the FPGA stream engine.
            # Leaves the ARM server running for fast restarts.
            self._scan_module.push_stream_stop()
            # Drain whatever the receiver buffered before it was stopped so the
            # consumer can still read the tail.
            self._drain_rx()
            s = self._rx.stats() if self._rx is not None else {}
            self.log.info(
                f'Stopped push stream. Total samples: {self._total_samples_acquired}, '
                f'gap(NaN) samples: {s.get("n_gap", 0)}, seq_skips: {s.get("n_seq_skips", 0)}'
            )
        except Exception as e:
            self.log.error(f'Error stopping stream: {e}')
            self._running = False
        finally:
            if self.module_state() == 'locked':
                self.module_state.unlock()

    # ----- KDC_HW_SYNC stream handover ------------------------------------------
    # The Red Pitaya streams ONE quantity at a time (single data3 ring, single
    # free-running sample counter selected by input_select). A marker-mode motor
    # scan therefore cannot run a *second* stream alongside this monitor stream --
    # doing so resets the shared sample counter under the monitor and corrupts both
    # (the bug that OOM'd the GUI). Instead the scan asks this single owner to hand
    # the stream over: the scan becomes the sole socket reader in marker mode, and
    # this module switches its own read path to a non-stealing *tap* on the scan's
    # feed -- so a live time-trace keeps showing the SAME samples the scan is
    # binning, with no second stream. end_scan_stream() restores the standalone
    # monitor. The FPGA frequency lock is unaffected throughout (it runs in
    # hardware and does not depend on the stream).

    def get_scan_module(self):
        """Return the shared pyrpl scan module (for marker register reads etc.)."""
        return self._scan_module

    @property
    def scan_active(self) -> bool:
        """True while a KDC_HW_SYNC scan owns the stream (monitor reads via a tap)."""
        return self._scan_active

    def begin_scan_stream(self, input_source='ftw_corr', ring_bytes=0, coalesce_us=0):
        """Hand the single board stream to a marker-mode (MODE 3) motor scan.

        The scan becomes the sole socket reader (marker mode) on ``input_source``.
        If a monitor stream (time-series / ODMR tracking) was running, this module
        seamlessly switches its read path to a tap on the scan's feed so the display
        continues uninterrupted. Returns the pyrpl scan module. Pair with
        end_scan_stream().
        """
        with self._thread_lock:
            if self._scan_module is None:
                raise RuntimeError('redpitaya_stream not activated; no scan module available')
            if input_source not in ('demod', 'ftw_corr'):
                self.log.warning("Scan stream input '%s' invalid; using 'ftw_corr'.",
                                 input_source)
                input_source = 'ftw_corr'
            # Remember whether a monitor was displaying, so we can restore a
            # standalone monitor stream afterwards.
            was_running = bool(self._running)
            self._scan_resume = was_running
            self._scan_resume_input = self._current_stream_input
            # Stop the monitor's OWN receiver (the scan will own the single socket);
            # keep module_state as-is -- we re-point _rx at the scan's tap below so a
            # running display does not skip a beat.
            if was_running:
                try:
                    self._scan_module.push_stream_stop()
                except Exception as e:  # noqa: BLE001
                    self.log.warning('Error stopping monitor receiver for handover: %s', e)
                self._rx = None
            # Start marker-mode streaming on the requested quantity. The scan reads
            # samples + markers from the same pyrpl scan module; we read a tap.
            try:
                self._current_stream_input = input_source
                self._scan_module.input_select = input_source
                self._scan_module.mapped_stream_start(
                    input_source=input_source,
                    ring_bytes=int(ring_bytes) if ring_bytes else int(self._stream_ring_bytes),
                    coalesce_us=int(coalesce_us) if coalesce_us else int(self._stream_coalesce_us))
            except Exception:
                # Don't leave the monitor stranded if the scan stream fails to start.
                self._scan_active = False
                if was_running:
                    try:
                        self._current_stream_input = self._scan_resume_input
                        if self.module_state() == 'locked':
                            self.module_state.unlock()
                        self._running = False
                        self.start_stream()
                        self.log.info('KDC_HW_SYNC: scan stream failed to start; '
                                      'restored the monitor stream.')
                    except Exception as e2:  # noqa: BLE001
                        self.log.error('KDC_HW_SYNC: scan stream failed AND monitor '
                                       'restore failed: %s', e2)
                self._scan_resume = False
                raise
            self._scan_active = True
            # If a display was running, attach a tap so it keeps reading seamlessly.
            if was_running:
                self._scan_tap = self._scan_module.add_stream_tap()
                self._rx = self._scan_tap
                self._pending = np.empty(0, dtype=np.float64)
                self._total_samples_acquired = 0
                self._last_gap_reported = 0
                self._running = True
            self.log.info("KDC_HW_SYNC scan now owns the push stream (marker mode, "
                          "input='%s'%s).", input_source,
                          '; time-trace tapped' if was_running else '')
            return self._scan_module

    def end_scan_stream(self):
        """Stop the scan's marker stream and restore the standalone monitor stream."""
        with self._thread_lock:
            if not self._scan_active:
                return
            self._scan_active = False
            # Was a display reading the scan's tap? (either resumed monitor or a
            # time-trace started during the scan)
            display_active = bool(self._running)
            resume_input = self._scan_resume_input if self._scan_resume else self._current_stream_input
            # Drop the tap and the scan's marker stream.
            if self._scan_tap is not None:
                try:
                    self._scan_module.remove_stream_tap(self._scan_tap)
                except Exception:  # noqa: BLE001
                    pass
                self._scan_tap = None
            self._rx = None
            self._running = False
            if self.module_state() == 'locked':
                self.module_state.unlock()
            try:
                self._scan_module.mapped_stream_stop()
            except Exception as e:  # noqa: BLE001
                self.log.warning('Error stopping KDC_HW_SYNC scan stream: %s', e)
            # Restore a standalone monitor stream if anything was (or should be) shown.
            if self._scan_resume or display_active:
                try:
                    self._current_stream_input = resume_input
                    self.start_stream()  # _scan_active is False -> real start
                    self.log.info("KDC_HW_SYNC: restored monitor stream (input='%s').",
                                  self._current_stream_input)
                except Exception as e:  # noqa: BLE001
                    self.log.error('Failed to restore monitor stream after scan: %s', e)
            self._scan_resume = False

    def _drain_rx(self):
        """Pull all samples the receiver has buffered, calibrate them, and append
        to the local FIFO. Non-blocking. Must be called under ``self._thread_lock``.

        Lost (NaN) samples are preserved so the time axis stays truthful; gap
        growth is surfaced via a warning. Returns the number of samples appended.
        """
        if self._rx is None:
            return 0
        try:
            # float64 already, with NaN where the FPGA overran the ARM drainer.
            raw_data = self._rx.read()

            if raw_data.size == 0:
                # Surface receiver-thread errors even when no data arrives.
                err = self._rx.error
                if err is not None:
                    self.log.warning(f'Stream receiver error: {err}')
                return 0

            # Apply calibration/conversion and scaling based on input mode.
            # NaN gap markers propagate cleanly through both paths.
            if self._current_stream_input == 'ftw_corr':
                # FTW correction: convert to Hz using pyrpl's conversion
                calibrated_data = self._scan_module.ftw_to_hz(raw_data) * self._signal_scale
            else:
                # Demod: apply calibration factor
                calibrated_data = raw_data * self._calibration_factor * self._signal_scale

            # Append to the local FIFO (no artificial cap -> no silent drops).
            self._pending = (np.concatenate((self._pending, calibrated_data))
                             if self._pending.size else calibrated_data)
            n_samples = calibrated_data.size
            self._total_samples_acquired += n_samples

            # Surface genuine FPGA-ring overruns (NaN-filled gaps) to the user.
            stats = self._rx.stats()
            n_gap = stats.get('n_gap', 0)
            if n_gap > self._last_gap_reported:
                new_gap = n_gap - self._last_gap_reported
                self._last_gap_reported = n_gap
                msg = (f'Stream overrun: {new_gap} lost samples NaN-filled '
                       f'(total gap {n_gap}). Read faster or reduce other load.')
                if self._streaming_mode == StreamingMode.FINITE:
                    self.log.error('FINITE mode data loss: ' + msg)
                else:
                    self.log.warning(msg)

            # Check if FINITE mode target reached.
            if self._streaming_mode == StreamingMode.FINITE:
                if self._total_samples_acquired >= self._channel_buffer_size:
                    self.log.debug('FINITE mode target reached, stopping stream')
                    self._do_stop()

            return n_samples

        except Exception as e:
            self.log.error(f'Error draining stream receiver: {e}')
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
            with self._thread_lock:
                # Pull whatever the receiver thread has buffered into the FIFO.
                self._drain_rx()

                if self._pending.size > 0:
                    to_read = min(self._pending.size,
                                  samples_per_channel - samples_read)
                    chunk = self._pending[:to_read]
                    self._pending = self._pending[to_read:]
                    data_buffer_flat[samples_read:samples_read + to_read] = chunk
                    samples_read += to_read

            if samples_read >= samples_per_channel:
                break

            # Check timeout
            if time.time() - start_time > timeout:
                raise TimeoutError(
                    f'Timeout waiting for {samples_per_channel} samples. '
                    f'Only {samples_read} samples acquired after {timeout:.1f}s'
                )

            # No data right now: yield briefly (the receiver thread keeps draining
            # the board in the background, so we never miss samples by sleeping).
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

        # Validate buffer
        data_buffer_flat = data_buffer.ravel()
        n_channels = len(self._active_channels)

        if data_buffer.dtype != self._constraints.data_type:
            raise TypeError(
                f'Buffer dtype {data_buffer.dtype} does not match required {self._constraints.data_type}'
            )

        max_samples = data_buffer_flat.size // n_channels

        with self._thread_lock:
            # Pull the latest data from the receiver thread into the FIFO.
            self._drain_rx()

            samples_to_read = min(self._pending.size, max_samples)
            if samples_to_read == 0:
                return 0

            chunk = self._pending[:samples_to_read]
            self._pending = self._pending[samples_to_read:]

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
            # Non-blocking: read all currently available
            with self._thread_lock:
                self._drain_rx()
                samples_per_channel = int(self._pending.size)
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
