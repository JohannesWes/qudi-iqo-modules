# -*- coding: utf-8 -*-
"""
Red Pitaya ODMR Frequency Lock Hardware Interface.

Wraps PyRPL's odmr_freq_lock module for Qudi integration.

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

import threading
from typing import Dict, Any, List, Optional, Tuple
import numpy as np
from qudi.core.configoption import ConfigOption
from qudi.interface.odmr_freq_lock_interface import OdmrFreqLockInterface
from qudi.interface.multi_resonance_tracking_interface import MultiResonanceTrackingInterface
from qudi.hardware.redpitaya.resource_manager import get_pyrpl_instance, release_pyrpl_instance


class RedPitayaOdmrLockHardware(OdmrFreqLockInterface, MultiResonanceTrackingInterface):
    """
    Hardware interface to Red Pitaya ODMR frequency lock via PyRPL.

    Implements both the single-resonance ``OdmrFreqLockInterface`` (wraps the
    region-8 lock loop ``rp.odmrfreqlock``) and the multi-resonance
    ``MultiResonanceTrackingInterface`` (additionally wraps the region-9
    oscillator/freeze ``rp.odmrmultitrack``, the per-slot integrators, and the
    region-5 ``rp.scan`` continuous-hop loop + push stream + hop markers). The
    overlapping method names (enable_lock, set_invert, set_max_correction_hz,
    set_bandwidth) have identical semantics across the two interfaces, so a single
    implementation satisfies both. The per-resonance SSB cal + LO jump-list live on
    the microwave/IF source module, not here.

    Config example:
        redpitaya_odmr_lock:
            module.Class: 'redpitaya.redpitaya_odmr_lock.RedPitayaOdmrLockHardware'
            options:
                redpitaya_config_name: 'rpy_shared_config'
                redpitaya_hostname: '10.203.129.28'
                lock_in_filter_resonance_1: '2kHz_minphase'
                lock_in_filter_resonance_2: '2kHz_minphase'
                max_trace_samples: 2000000   # cap on the high-rate trace buffer
    """

    _redpitaya_config_name = ConfigOption('redpitaya_config_name',
                                          default='rpy_shared_config', missing='info')
    _redpitaya_hostname = ConfigOption('redpitaya_hostname', missing='error')
    _lock_in_filter_resonance_1 = ConfigOption(
        'lock_in_filter_resonance_1', default='2kHz_minphase', missing='info')
    _lock_in_filter_resonance_2 = ConfigOption(
        'lock_in_filter_resonance_2', default='2kHz_minphase', missing='info')
    # Size (in stream WORDS) of the high-rate display buffer for read_traces. It is a
    # ROLLING window: once full, the oldest samples are dropped so the tracking GUI
    # shows the most recent slice at CONSTANT resolution (like the Time Series GUI),
    # instead of growing unbounded / freezing. In the current marked stream 4 words
    # = 1 sample (~30.5 kHz), so 2 MWords ~= 500k samples ~= 16.4 s window. Reduce for a shorter,
    # finer-resolution window. Indefinite drift is still fully covered by the low-rate
    # per-slot register polling (get_slot_status / correction-history plot).
    _max_trace_samples = ConfigOption('max_trace_samples', default=2_000_000, missing='nothing')

    # ~30.5 kHz demod/stream sample rate (125 MHz / 4096 CIC decimation)
    _STREAM_SAMPLE_RATE_HZ = 125e6 / 4096

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pyrpl = None
        self._lock = None
        self._multitrack = None
        self._fgen3 = None
        self._scan = None
        # multi-resonance tracking state
        self._tracking_active = False
        self._streaming_traces = False
        self._single_slot_no_hop = False
        self._nslots = 2
        self._trace_values = np.array([], dtype=np.float64)
        self._trace_ticks = np.array([], dtype=np.int64)
        self._trace_steps = np.array([], dtype=np.int64)
        # Number of complete stream records removed from the front of the rolling
        # buffer.  This preserves an elapsed-since-stream-start time axis.
        self._trace_sample_offset = 0
        self._stream_words_per_sample = 3  # legacy-safe until the FPGA reports 4
        # Runtime rolling-window size in WORDS for the high-rate display buffer
        # (settable live via set_trace_window_seconds; initialized from the
        # max_trace_samples ConfigOption in on_activate).
        self._trace_window_words = 2_000_000
        self._stream_source = 'marked'
        self._configured_lock_in_filters = ['2kHz_minphase', '2kHz_minphase']
        # 2D motor-scan takeover: while True the motor scan owns the physical stream
        # drain (read_stream_words), so read_traces must NOT also drain (word theft).
        self._mapped_scan_active = False
        # Single-position field-trace takeover: while True the tracking logic's
        # continuous field logger owns the physical drain (read_stream_words), so
        # read_traces must NOT also drain (same word-theft guard as the mapped scan).
        self._field_drain_active = False
        # Serializes stream-touching ops (drain + restart) across the two caller
        # threads: the tracking logic's read_traces poll and the motor-scan logic's
        # read_stream_words / enable_position_markers restart. Qudi hardware methods
        # run on the caller's thread (no auto-marshalling), so without this a restart
        # could replace the StreamClient mid-read.
        self._stream_lock = threading.RLock()

    def on_activate(self):
        """Connect to PyRPL and get the lock / multitrack / fgen3 / scan modules."""
        # Get shared PyRPL instance via resource manager
        self._pyrpl, _ = get_pyrpl_instance(
            hostname=self._redpitaya_hostname,
            config_name=self._redpitaya_config_name
        )

        # Runtime high-rate display window (words), seeded from the ConfigOption.
        self._trace_window_words = max(4, int(self._max_trace_samples))

        rp = self._pyrpl.rp
        # Get odmrfreqlock module (PyRPL naming: all lowercase, no underscores)
        self._lock = rp.odmrfreqlock
        # Multi-resonance hardware surface (may be absent on an old bitstream)
        self._multitrack = getattr(rp, 'odmrmultitrack', None)
        self._fgen3 = getattr(rp, 'fgen3', None)
        self._scan = getattr(rp, 'scan', None)
        if self._scan is not None:
            self._stream_words_per_sample = int(self._scan.stream_words_per_sample)
            if self._stream_words_per_sample != 4:
                self.log.warning(
                    'FPGA reports the legacy %d-word tracking stream; the pre-FIR CIC '
                    'trace is unavailable until the new bitstream is loaded.',
                    self._stream_words_per_sample)

        # Multitrack routes resonance 1 through lockin channel 1 and resonance 2
        # through lockin1 channel 1. Configure both instances explicitly.
        self._configure_lock_in_filters(rp)

        # Ensure lock + oscillator are disabled on activation
        self._lock.enable = False
        if self._multitrack is not None:
            try:
                self._multitrack.enable = False
            except Exception as e:
                self.log.warning(f'Could not disable multitrack oscillator on activation: {e}')

        if self._multitrack is None:
            self.log.warning('rp.odmrmultitrack not found - multi-resonance tracking '
                             'unavailable (old bitstream?). Single-resonance lock still works.')
        self.log.info(f'Red Pitaya ODMR Lock connected: {self._redpitaya_hostname}')

    def _configure_lock_in_filters(self, rp) -> None:
        """Apply the per-resonance FIR phase selection to both lock-in instances."""
        valid_filters = {'2kHz_minphase', '2kHz_linear', '2kHz'}
        requested = [self._lock_in_filter_resonance_1,
                     self._lock_in_filter_resonance_2]

        for index, (module_name, filter_name) in enumerate(
                zip(('lockin', 'lockin1'), requested), start=1):
            if filter_name not in valid_filters:
                self.log.warning(
                    f'Invalid lock-in filter for resonance {index}: "{filter_name}"; '
                    'using "2kHz_minphase"')
                filter_name = '2kHz_minphase'

            lock_in = getattr(rp, module_name, None)
            if lock_in is None:
                self.log.warning(
                    f'rp.{module_name} not found; cannot configure resonance {index} FIR')
                continue

            # Channel 1 is the current multitrack data path. Keep channel 2 in
            # step so it is ready if the unused quadrature lane is enabled later.
            lock_in.filter_select_ch1 = filter_name
            lock_in.filter_select_ch2 = filter_name
            self._configured_lock_in_filters[index - 1] = (
                '2kHz_minphase' if filter_name == '2kHz' else filter_name)
            self.log.info(
                f'Resonance {index} FIR ({module_name}): {filter_name}')

    def get_trace_calibration(self) -> Dict[str, Any]:
        """Return the fixed-point scaling needed to calibrate the CIC trace.

        ``fir_dc_gain_from_cic_lsb`` maps the exported CIC word (CIC[39:8]) to
        the selected 32-bit FIR output at DC.  The discriminator fit supplies the
        remaining physical LSB/Hz factor in the tracking logic.
        """
        minphase_gain = 937716.0 / (2 ** 21)
        linear_gain = (802861.0 / (2 ** 21)) * (299.0 / 256.0)
        gains = [linear_gain if name == '2kHz_linear' else minphase_gain
                 for name in self._configured_lock_in_filters]
        return {
            'stream_words_per_sample': int(self._stream_words_per_sample),
            'filters': tuple(self._configured_lock_in_filters[:self._nslots]),
            'fir_dc_gain_from_cic_lsb': tuple(gains[:self._nslots]),
        }

    def on_deactivate(self):
        """Disable lock and disconnect."""
        try:
            if self._tracking_active:
                self.stop_tracking()
        except Exception as e:
            self.log.warning(f'Could not stop multi-resonance tracking on deactivation: {e}')
        if self._lock is not None:
            try:
                self._lock.enable = False
            except Exception as e:
                self.log.warning(f'Could not disable lock on deactivation: {e}')
        if self._multitrack is not None:
            try:
                self._multitrack.enable = False
            except Exception as e:
                self.log.warning(f'Could not disable multitrack on deactivation: {e}')

        # Release pyrpl instance
        if self._pyrpl is not None:
            release_pyrpl_instance(
                hostname=self._redpitaya_hostname,
                config_name=self._redpitaya_config_name
            )
            self._pyrpl = None

        self._lock = None
        self.log.info('Red Pitaya ODMR Lock deactivated')

    # =========================================================================
    # OdmrFreqLockInterface Implementation
    # =========================================================================

    def set_bandwidth(self, bandwidth_hz: float, slope_lsb_per_hz: float,
                      pi: bool = False, zero_ratio: float = 3.0) -> None:
        """Configure the loop gains (integral-only, or PI if ``pi=True``).

        Satisfies both OdmrFreqLockInterface (single-res, 2 args) and
        MultiResonanceTrackingInterface (adds ``pi``/``zero_ratio``). The gains are
        global (shared by all slots in hardware).
        """
        if bandwidth_hz <= 0:
            raise ValueError(f'Bandwidth must be positive, got {bandwidth_hz}')
        if slope_lsb_per_hz <= 0:
            raise ValueError(f'Slope must be positive, got {slope_lsb_per_hz}')

        if pi:
            if not 0.1 <= zero_ratio <= 30.0:
                raise ValueError(f'Zero ratio must be in [0.1, 30.0], got {zero_ratio}')
            self._lock.set_bandwidth_pi(bandwidth_hz, slope_lsb_per_hz, zero_ratio)
            self.log.info(
                f'Lock configured (PI): BW={bandwidth_hz:.1f} Hz, '
                f'slope={slope_lsb_per_hz:.3e} LSB/Hz, alpha={zero_ratio:.2f}'
            )
        else:
            self._lock.set_bandwidth(bandwidth_hz, slope_lsb_per_hz)
            self.log.info(
                f'Lock configured (I-only): BW={bandwidth_hz:.1f} Hz, '
                f'slope={slope_lsb_per_hz:.3e} LSB/Hz'
            )

    def set_bandwidth_pi(self, bandwidth_hz: float, slope_lsb_per_hz: float,
                         zero_ratio: float = 3.0) -> None:
        """Configure PI frequency lock with zero placement."""
        if bandwidth_hz <= 0:
            raise ValueError(f'Bandwidth must be positive, got {bandwidth_hz}')
        if slope_lsb_per_hz <= 0:
            raise ValueError(f'Slope must be positive, got {slope_lsb_per_hz}')
        if not 0.1 <= zero_ratio <= 30.0:
            raise ValueError(f'Zero ratio must be in [0.1, 30.0], got {zero_ratio}')

        self._lock.set_bandwidth_pi(bandwidth_hz, slope_lsb_per_hz, zero_ratio)

        self.log.info(
            f'Lock configured (PI): BW={bandwidth_hz:.1f} Hz, '
            f'slope={slope_lsb_per_hz:.3e} LSB/Hz, α={zero_ratio:.2f} '
            f'(zero at {bandwidth_hz/zero_ratio:.1f} Hz)'
        )

    def enable_lock(self, enable: bool) -> None:
        """Enable or disable frequency lock."""
        self._lock.enable = bool(enable)
        self.log.info(f'Lock {"enabled" if enable else "disabled"}')

    def get_status(self) -> Dict[str, Any]:
        """Get current lock status."""
        return self._lock.get_status()

    def clear(self) -> None:
        """Clear integrator state."""
        self._lock.clear()
        self.log.debug('Lock integrator cleared')

    def get_constraints(self) -> Dict[str, Any]:
        """Get hardware constraints."""
        # PyRPL constraints (from odmr_freq_lock.py and Verilog)
        return {
            'bandwidth_range': (10.0, 10000.0),  # Hz, practical range
            'slope_range': (1e-6, 1e6),  # LSB/Hz, very wide range
            'damping_range': (0.5, 1.0),  # Dimensionless
            'max_correction_hz': 62.5e6,  # Half of 125 MHz (Nyquist)
        }

    def set_max_correction_hz(self, max_correction_hz: float) -> None:
        """Set maximum frequency correction (FTW saturation limit)."""
        if max_correction_hz <= 0:
            raise ValueError(f'Max correction must be positive, got {max_correction_hz}')
        if max_correction_hz > 62.5e6:
            raise ValueError(f'Max correction cannot exceed 62.5 MHz (Nyquist), got {max_correction_hz}')

        self._lock.max_correction_hz = max_correction_hz

        self.log.info(f'Lock max correction set to {max_correction_hz/1e6:.3f} MHz')

    def get_max_correction_hz(self) -> float:
        """Get current maximum frequency correction setting."""
        return self._lock.max_correction_hz

    def set_invert(self, inverted: bool) -> None:
        """
        Set error signal polarity inversion.

        For LSB (f_RF = f_LO - f_IF): set inverted=True
        For USB (f_RF = f_LO + f_IF): set inverted=False
        """
        self._lock.invert = bool(inverted)
        sideband = 'LSB' if inverted else 'USB'
        self.log.info(f'Error inversion set to {inverted} (for {sideband} operation)')

    def get_invert(self) -> bool:
        """Get current error signal inversion setting."""
        return self._lock.invert

    # =========================================================================
    # MultiResonanceTrackingInterface Implementation
    # =========================================================================

    def _require_multitrack(self):
        if self._multitrack is None:
            raise RuntimeError('Multi-resonance tracking unavailable: rp.odmrmultitrack '
                               'not present (old bitstream?).')

    def _configure_hop_trigger_pin(self):
        """Route the scan LO-hop trigger to DIO7_P as an inverted (idle-high,
        active-low) module output, matching the Windfreak active-low trigger input."""
        try:
            self._pyrpl.rp.hk.configure_pin('P7', direction='output',
                                            source='module', invert=True)
            self.log.debug('DIO7_P configured as inverted scan-trigger output for LO hopping')
        except Exception as e:
            self.log.warning(f'Could not configure DIO7 hop-trigger pin: {e}')

    @property
    def nslots(self) -> int:
        """Number of resonance slots N implemented in hardware."""
        try:
            return int(self._lock.nslots)
        except Exception:
            return int(self._nslots)

    @property
    def tracking_engine_active(self) -> bool:
        """True while the continuous acquisition/hop engine is running."""
        return bool(self._tracking_active)

    @property
    def trace_stream_active(self) -> bool:
        """True while the simultaneous error/correction stream is running."""
        return bool(self._streaming_traces)

    @property
    def position_markers_active(self) -> bool:
        """True while a mapped motor scan owns the physical stream drain."""
        return bool(self._mapped_scan_active)

    @property
    def field_drain_active(self) -> bool:
        """True while a single-position field logger owns the physical stream drain."""
        return bool(self._field_drain_active)

    @property
    def stream_source(self) -> str:
        """Current reconstruction source: 'marked' (fresh-only, per-visit exact) or
        the legacy ZOH-filled 'dual'."""
        return getattr(self, '_stream_source', 'marked')

    @property
    def stream_sample_rate(self) -> float:
        """Aggregate demod stream rate [Hz] (125 MHz / 4096 ~= 30.5 kHz)."""
        return float(self._STREAM_SAMPLE_RATE_HZ)

    @property
    def stream_words_per_sample(self) -> int:
        """Number of 32-bit words in one synchronized stream record."""
        return int(self._stream_words_per_sample)

    def begin_field_drain(self) -> None:
        """Hand the physical stream drain to a single-position field logger.

        Mirrors the 2D mapped-scan takeover: while active, ``read_traces`` (the 5 Hz
        GUI poll) stops draining so the field logger's ``read_stream_words`` sees every
        word. The logger still feeds the display buffer (read_stream_words does), so
        the tracking GUI's live traces keep updating. Requires an active trace stream.
        """
        with self._stream_lock:
            if not self._streaming_traces:
                raise RuntimeError('Start the trace stream before field logging.')
            if self._mapped_scan_active:
                raise RuntimeError('A mapped 2D scan already owns the stream drain.')
            self._field_drain_active = True

    def end_field_drain(self) -> None:
        """Return the drain to the GUI poll (idempotent)."""
        with self._stream_lock:
            self._field_drain_active = False

    def get_iq_demod_phase(self) -> float:
        """Current iq0 demod phase [deg] (the legacy/scan reference phase you tune
        visually on the ODMR curve). The oscillator's demod_phase uses the SAME sign
        convention (odmr_multitrack.demod_phase is invert=True like iq0.phase), so
        set the oscillator demod_phase equal to this to reproduce the scan demod."""
        try:
            return float(self._pyrpl.rp.iq0.phase)
        except Exception as e:
            self.log.warning(f'Could not read iq0 phase: {e}')
            return 0.0

    def configure_oscillator(self, f_m_hz: float, demod_phase_deg: float,
                             settle_time_s: float, source: str = 'current_step') -> None:
        """Configure the per-channel modulation oscillator and freeze window."""
        self._require_multitrack()
        if source not in ('current_step', 'sw'):
            raise ValueError(f"source must be 'current_step' or 'sw', got {source}")
        self._multitrack.frequency = float(f_m_hz)
        self._multitrack.demod_phase = float(demod_phase_deg)
        self._multitrack.settle_time = float(settle_time_s)
        self._multitrack.src = source
        if source == 'sw':
            try:
                self._multitrack.sw_channel = 0
            except Exception:
                pass
        self.log.info(
            f'Oscillator configured: f_m={f_m_hz/1e3:.4f} kHz, '
            f'demod_phase={demod_phase_deg:.1f} deg, settle={settle_time_s*1e6:.0f} us, '
            f'src={source}'
        )

    def enable_oscillator(self, enable: bool) -> None:
        """Enable/disable the multi-channel oscillator + freeze gating."""
        self._require_multitrack()
        self._multitrack.enable = bool(enable)
        self.log.info(f'Multitrack oscillator {"enabled" if enable else "disabled"}')

    def set_integrator_source(self, source: str = 'current_step') -> None:
        """Select which slot integrates / which cal slot is active: hardware hop
        index ('current_step') or software ('sw'). Applies to both the per-slot
        integrators (odmrfreqlock) and the cal-slot bank (fgen3)."""
        if source not in ('current_step', 'sw'):
            raise ValueError(f"source must be 'current_step' or 'sw', got {source}")
        hw = (source == 'current_step')
        self._lock.active_slot_src = hw
        if not hw:
            try:
                self._lock.active_slot = 0
            except Exception:
                pass
        if self._fgen3 is not None:
            self._fgen3.active_slot_src = hw
            if not hw:
                try:
                    self._fgen3.active_slot = 0
                except Exception:
                    pass
        self.log.info(f'Integrator + cal-slot source set to {source} '
                      f'(active_slot_src={hw})')

    def clear_integrators(self) -> None:
        """Clear all per-slot integrator states."""
        self._lock.clear()
        self.log.debug('All per-slot integrators cleared')

    def get_slot_status(self, slot: int) -> Dict[str, Any]:
        """Per-slot lock status (enabled/locked/saturated/error_lsb/correction_hz)."""
        n = self.nslots
        if not (0 <= slot < n):
            raise ValueError(f'slot must be in 0..{n-1}, got {slot}')
        status = dict(self._lock.status_slot(slot))
        # Ensure the headline fields are present with consistent names
        status.setdefault('correction_hz', self._lock.correction_hz_slot(slot))
        status.setdefault('error_lsb', self._lock.error_lsb_slot(slot))
        status['slot'] = slot
        return status

    def get_all_status(self) -> List[Dict[str, Any]]:
        """Per-slot status for all slots. Built from get_slot_status so the
        correction_hz / error_lsb / locked / saturated keys are always present
        (the GUI reads these)."""
        return [self.get_slot_status(s) for s in range(self.nslots)]

    def start_tracking(self, nslots: int, dwell_time_s: float,
                       settling_time_s: float, trigger_length_s: float,
                       stream_traces: bool = True) -> None:
        """Start indefinite hardware-driven LO hopping over ``nslots`` resonances."""
        self._require_multitrack()
        if self._scan is None:
            raise RuntimeError('rp.scan not available; cannot start hopping.')
        if self._tracking_active:
            self.log.warning('Multi-resonance tracking already active; ignoring start.')
            return

        self._nslots = int(nslots)
        # ensure the LO-hop trigger reaches the Windfreak (DIO7_P, inverted)
        if self._nslots > 1:
            self._configure_hop_trigger_pin()
        # reset the high-rate trace buffers
        self._trace_values = np.array([], dtype=np.float64)
        self._trace_ticks = np.array([], dtype=np.int64)
        self._trace_steps = np.array([], dtype=np.int64)
        self._trace_sample_offset = 0
        self._streaming_traces = False

        self._single_slot_no_hop = self._nslots == 1
        if self._single_slot_no_hop:
            self.set_integrator_source('sw')
            try:
                if self._scan.busy:
                    self._scan.stop()
                self._scan.reset()
                self._scan.num_steps = 1
            except Exception as e:
                self.log.warning(f'Could not reset scan current_step for N=1 tracking: {e}')
            if self._fgen3 is not None:
                try:
                    self._fgen3.active_slot_src = False
                    self._fgen3.active_slot = 0
                except Exception:
                    pass

        # Mark the engine active before starting its two independent components so
        # a caller can reliably use stop_tracking() to clean up a partial failure.
        self._tracking_active = True

        if stream_traces:
            # MARKED-continuous self-describing push stream:
            # a free-running /4096 tick streams [err, corr, state] EVERY demod period,
            # where state = resonance index when live or a DEAD sentinel during the
            # per-hop settle/freeze. So the dead-time is explicit and the PC timeline is
            # uniform/exact -> per-resonance reconstruction has correct absolute timing
            # (the legacy 'dual' mode dropped the dead-time and inflated the visit rate).
            # Decode with scan.reconstruct_marked_series(). Requires the marked-stream
            # bitstream (STREAM_CONTROL[5]); falls back to 'dual' if unavailable.
            self._start_trace_stream_hardware()

        if not self._single_slot_no_hop:
            # Hopping is independent of streaming. Starting it separately is what
            # lets stop_trace_stream() leave N=2 tracking running.
            self._scan.num_steps = self._nslots
            self._scan.dwell_time = dwell_time_s
            self._scan.settling_time = settling_time_s
            self._scan.trigger_length = trigger_length_s
            self._scan.start(continuous=True)

        self._tracking_active = True
        self.log.info(
            f'Multi-resonance tracking started: N={self._nslots}, '
            f'dwell={dwell_time_s*1e6:.0f} us, settle={settling_time_s*1e6:.0f} us, '
            f'stream_traces={stream_traces}'
            + (' (single-slot no-hop: dwell/settle ignored)' if self._single_slot_no_hop else '')
        )

    def stop_tracking(self) -> None:
        """Stop continuous hopping (and the trace stream if running)."""
        if not self._tracking_active:
            return
        try:
            with self._stream_lock:
                if self._streaming_traces:
                    self._scan.hop_stream_stop()
                    self._streaming_traces = False
                if not self._single_slot_no_hop:
                    self._scan.stop()
        finally:
            self._tracking_active = False
            self._streaming_traces = False
            self._single_slot_no_hop = False
            self._mapped_scan_active = False
        self.log.info('Multi-resonance tracking stopped')

    def _start_trace_stream_hardware(self) -> None:
        """Start/reset only the push-stream side of the tracking engine."""
        self._trace_values = np.array([], dtype=np.float64)
        self._trace_ticks = np.array([], dtype=np.int64)
        self._trace_steps = np.array([], dtype=np.int64)
        self._trace_sample_offset = 0
        self._stream_source = 'marked'
        try:
            self._scan.hop_stream_start(input_source='marked')
        except Exception as e:
            self.log.warning(f"Marked-continuous stream unavailable ({e}); "
                             f"falling back to legacy 'dual'.")
            self._stream_source = 'dual'
            self._scan.hop_stream_start(input_source='dual')
        self._stream_words_per_sample = int(self._scan.stream_words_per_sample)
        if self._stream_words_per_sample != 4:
            self.log.warning('Tracking stream has %d words/sample; CIC trace will be NaN.',
                             self._stream_words_per_sample)
        self._streaming_traces = True

    def start_trace_stream(self) -> None:
        """Start traces without changing hopping or the FPGA lock state."""
        if not self._tracking_active:
            raise RuntimeError('Start the hopping/acquisition engine before streaming.')
        if self._streaming_traces:
            self.log.warning('Multi-resonance trace stream already active.')
            return
        with self._stream_lock:
            self._start_trace_stream_hardware()
        self.log.info('Multi-resonance trace stream started independently.')

    def stop_trace_stream(self) -> None:
        """Stop traces without changing hopping or the FPGA lock state."""
        with self._stream_lock:
            # Check ownership under the same lock used by enable_position_markers;
            # otherwise a concurrent GUI stop could slip between marker takeover
            # and the stream restart.
            if not self._streaming_traces:
                return
            if self._mapped_scan_active:
                raise RuntimeError(
                    'Cannot stop the trace stream while a KDC_HW_SYNC_MULTIRES motor '
                    'scan owns its position-marker data.')
            self._scan.hop_stream_stop()
            self._streaming_traces = False
            self._mapped_scan_active = False
        self.log.info('Multi-resonance trace stream stopped independently.')

    def read_traces(self) -> Optional[Dict[str, Any]]:
        """Reconstructed per-resonance high-rate traces since session start (capped).

        Decodes the self-describing stream ([err, corr, cic, state] records)
        into synchronous per-resonance post-FIR error, correction, and pre-FIR CIC traces
        at the full demod rate (~30.5 kHz aggregate, shared across resonances by the
        hop schedule). Indefinite operation is covered by per-slot register polling
        (:meth:`get_slot_status`); this high-rate buffer is a ROLLING window of the
        last ``max_trace_samples`` words (oldest dropped), so it stays bounded at
        constant resolution instead of growing/freezing.

        Returns:
            dict or None: ``{'times': (T,), 'err': (N, T) raw LSB,
            'corr_hz': (N, T) Hz, 'sample_rate': Hz}`` where T is the number of
            complete records received and N is the number of resonances.
        """
        if not (self._tracking_active and self._streaming_traces):
            return None

        # Pull new words; the FPGA-reported record width keeps all columns aligned.
        # While a mapped 2D scan owns the drain, it feeds self._trace_values via
        # read_stream_words(); draining here too would steal words from it, so skip
        # the physical drain and just reconstruct the shared buffer's current state.
        if not self._mapped_scan_active and not self._field_drain_active:
            with self._stream_lock:
                new_vals = self._scan.hop_stream_read()  # float64, NaN = transport loss
            if new_vals is not None and len(new_vals):
                self._trace_values = np.concatenate([self._trace_values,
                                                     np.asarray(new_vals, dtype=np.float64)])
                self._trim_trace_buffer()

        width = int(self._stream_words_per_sample)
        if self._trace_values.size < width:
            return None

        if getattr(self, '_stream_source', 'dual') == 'marked':
            # Marked-continuous: every demod period is a slot, dead-time explicit ->
            # uniform/exact timeline. reconstruct returns fresh-while-live (NaN during
            # dead or another resonance); ZOH-fill here so the GUI shows continuous
            # per-resonance traces (the field estimate is held while parked).
            rec = self._scan.reconstruct_marked_series(
                self._trace_values, nslots=self._nslots, to_hz_corr=True,
                words_per_sample=width)
            err = self._ffill_marked_rows(rec['err'], rec['step'])
            corr_hz = self._ffill_marked_rows(rec['corr'], rec['step'])
            cic = self._ffill_marked_rows(rec['cic'], rec['step'])
            dead = rec.get('dead')
        else:
            rec = self._scan.reconstruct_dual_hop_series(
                self._trace_values, nslots=self._nslots, to_hz_corr=True,
                words_per_sample=width)
            err = rec['err']        # (N, T) raw LSB
            corr_hz = rec['corr']   # (N, T) Hz
            cic = rec['cic']        # (N, T) raw CIC LSB
            dead = None
        t_count = err.shape[1]
        times = (self._trace_sample_offset + np.arange(t_count, dtype=np.float64)) \
                / self._STREAM_SAMPLE_RATE_HZ
        out = {'times': times, 'err': err, 'corr_hz': corr_hz, 'cic': cic,
               'sample_rate': self._STREAM_SAMPLE_RATE_HZ}
        if dead is not None:
            out['dead'] = dead
        return out

    @staticmethod
    def _ffill_rows(a):
        """Row-wise forward-fill of NaNs (zero-order hold) for (N, T) arrays."""
        a = np.asarray(a, dtype=np.float64)
        if a.ndim != 2 or a.size == 0:
            return a
        out = a.copy()
        for r in range(out.shape[0]):
            row = out[r]
            valid = ~np.isnan(row)
            if not valid.any():
                continue
            idx = np.where(valid, np.arange(row.size), 0)
            np.maximum.accumulate(idx, out=idx)
            out[r] = row[idx]
        return out

    @classmethod
    def _ffill_marked_rows(cls, a, step):
        """Hold parked/dead slots while preserving loss in a resonance's live slot."""
        source = np.asarray(a, dtype=np.float64)
        out = cls._ffill_rows(source)
        step = np.asarray(step, dtype=np.int64)
        for r in range(out.shape[0]):
            live_loss = (step == r) & np.isnan(source[r])
            out[r, live_loss] = np.nan
        return out

    def _trim_trace_buffer(self) -> None:
        """Roll the high-rate display buffer to the last ``_trace_window_words`` words.

        Trims on a record boundary (word 0 = session start is aligned, so
        dropping a multiple of the reported width keeps the columns
        phase intact). Keeps the tracking GUI window bounded + constant-resolution
        instead of growing unbounded and freezing at a cap.
        """
        window = int(self._trace_window_words)
        width = int(self._stream_words_per_sample)
        n_complete = self._trace_values.size // width
        keep = window // width
        if keep >= 1 and n_complete > keep:
            dropped = n_complete - keep
            start = dropped * width
            self._trace_values = self._trace_values[start:]
            self._trace_sample_offset += dropped

    def set_trace_window_seconds(self, seconds: float) -> None:
        """Set the high-rate display window duration (rolling buffer length).

        Live-settable from the tracking GUI. In the marked stream one fixed-width
        at ~30.5 kHz, so the buffer holds ``seconds * rate`` samples; shrinking trims
        immediately on the next read, growing fills over ``seconds``. Independent of
        the per-slot register-poll drift history (that has its own length).
        """
        s = float(seconds)
        if not np.isfinite(s) or s <= 0:
            self.log.warning('set_trace_window_seconds: ignoring non-positive value %r', seconds)
            return
        width = int(self._stream_words_per_sample)
        self._trace_window_words = max(width,
                                       int(round(s * self._STREAM_SAMPLE_RATE_HZ)) * width)
        # Trim right away if the new window is shorter than the current buffer.
        self._trim_trace_buffer()
        self.log.debug('High-rate trace window set to %.2f s (%d words).',
                       s, self._trace_window_words)

    # =========================================================================
    # 2D motor-scan composition: x/y position markers on the running stream
    # =========================================================================
    def enable_position_markers(self, enable: bool) -> None:
        """Enable/disable KDC x/y position-marker capture on the running stream.

        Both directions RESTART the push stream (via ``hop_stream_start`` with/without
        ``xy_markers``), which resets the demod ring + sample counter so demod word 0
        aligns with the marker origin -- the alignment a 2D scan needs to bin the
        reconstructed traces by absolute marker index. The continuous-hop FSM and the
        per-slot lock integrators are separate from the stream engine, so this does
        NOT perturb the lock (only the high-rate display buffer restarts).
        """
        if self._scan is None:
            self.log.warning('enable_position_markers: rp.scan unavailable.')
            return
        with self._stream_lock:
            if enable and not (self._tracking_active and self._streaming_traces):
                raise RuntimeError(
                    'KDC_HW_SYNC_MULTIRES needs an active multi-resonance trace stream. '
                    'Configure tracking and click Start Stream first; Start Tracking is '
                    'optional (open-loop error mapping is supported).')
            if enable and getattr(self, '_stream_source', 'marked') != 'marked':
                self.log.warning(
                    "Position-marker 2D scan needs the MARKED stream for fresh-only "
                    "per-resonance binning; current source is %r. Per-bin means may "
                    "include held (parked) values.", self._stream_source)
            if enable:
                # Stop the tracking poll's drain FIRST (so it can't read the
                # StreamClient we are about to replace), then restart with markers.
                self._mapped_scan_active = True
                self._restart_mapped_stream(xy_markers=True)
            else:
                self._restart_mapped_stream(xy_markers=False)
                self._mapped_scan_active = False
        self.log.info('KDC x/y position markers %s (stream restarted, aligned).',
                      'ENABLED (2D mapped scan)' if enable else 'disabled')

    def _restart_mapped_stream(self, xy_markers: bool) -> None:
        """Restart the tracker's push stream with/without x/y marker capture.

        Uses ``hop_stream_start`` (which pulses a stream reset and starts a fresh
        receiver) so the new session starts at demod word 0. The hop FSM keeps
        looping (it is not restarted here). Also clears the display-trace buffer so
        :meth:`read_traces` re-aligns to the new word 0.
        """
        src = getattr(self, '_stream_source', 'marked')
        # Stop our own running stream cleanly first, so the (intentional) restart does
        # not trip pyrpl's "a push stream is already running" takeover warning -- that
        # warning should fire only for a genuine second, competing stream owner.
        try:
            self._scan.hop_stream_stop()
        except Exception:
            pass
        try:
            self._scan.hop_stream_start(input_source=src, xy_markers=bool(xy_markers))
        except TypeError:
            # older pyrpl without the xy_markers kwarg
            if xy_markers:
                self.log.error('pyrpl scan.hop_stream_start lacks xy_markers=; update '
                               'pyrpl to the 2D multi-resonance build.')
            self._scan.hop_stream_start(input_source=src)
        # fresh alignment for the display buffer
        # Preserve elapsed time across this intentional stream restart.  The exact
        # restart latency is not represented by FPGA samples, but the saved/visible
        # time axis never jumps backwards to zero.
        self._trace_sample_offset += self._trace_values.size // int(self._stream_words_per_sample)
        self._trace_values = np.array([], dtype=np.float64)

    def read_position_markers(self) -> Tuple[np.ndarray, np.ndarray]:
        """New (x, y) position markers since the last call, as sample indices."""
        empty = (np.array([], dtype=np.int64), np.array([], dtype=np.int64))
        if self._scan is None or not self._streaming_traces:
            return empty
        try:
            xm = np.asarray(self._scan.read_x_markers(), dtype=np.int64)
            ym = np.asarray(self._scan.read_y_markers(), dtype=np.int64)
        except Exception as e:
            self.log.warning('read_position_markers failed: %s', e)
            return empty
        # In the self-describing marked/dual stream the marker values are WORD
        # indices; convert with the FPGA-reported width to time-sample indices so
        # they index the reconstructed per-resonance traces directly.
        if getattr(self, '_stream_source', 'marked') in ('marked', 'dual'):
            xm = self._scan.markers_to_sample_index(
                xm, words_per_sample=self._stream_words_per_sample)
            ym = self._scan.markers_to_sample_index(
                ym, words_per_sample=self._stream_words_per_sample)
        return xm, ym

    def read_stream_words(self) -> np.ndarray:
        """New raw self-describing stream words (destructive drain)."""
        if self._scan is None or not self._streaming_traces:
            return np.array([], dtype=np.float64)
        try:
            with self._stream_lock:
                w = self._scan.hop_stream_read()
        except Exception as e:
            self.log.warning('read_stream_words failed: %s', e)
            return np.array([], dtype=np.float64)
        w = np.asarray(w, dtype=np.float64) if w is not None else np.array([], dtype=np.float64)
        # Keep the tracking-display buffer (read_traces) fed with recent data while
        # the motor scan owns the physical drain, so the tracking GUI's high-rate
        # plots still update -- rolling window, same as read_traces.
        if w.size:
            self._trace_values = np.concatenate([self._trace_values, w])
            self._trim_trace_buffer()
        return w

    def reconstruct_mapped_traces(self, words) -> Dict[str, Any]:
        """Decode accumulated record words -> fresh-only err/corr/CIC traces."""
        words = np.asarray(words, dtype=np.float64)
        rate = self._STREAM_SAMPLE_RATE_HZ
        if getattr(self, '_stream_source', 'marked') == 'marked':
            # Marked-continuous: fresh-only (NaN when parked/dead/loss). Exactly what
            # per-bin nanmean needs -- no zero-order hold leaking parked values.
            rec = self._scan.reconstruct_marked_series(
                words, nslots=self._nslots, to_hz_corr=True,
                words_per_sample=self._stream_words_per_sample)
        else:
            # Dual fallback: this ZOH-fills parked regions, so per-bin means may
            # include held values (warned in enable_position_markers).
            rec = self._scan.reconstruct_dual_hop_series(
                words, nslots=self._nslots, to_hz_corr=True,
                words_per_sample=self._stream_words_per_sample)
        err = np.asarray(rec['err'], dtype=np.float64)
        corr = np.asarray(rec['corr'], dtype=np.float64)
        cic = np.asarray(rec['cic'], dtype=np.float64)
        t_count = err.shape[1] if err.ndim == 2 else 0
        times = np.arange(t_count, dtype=np.float64) / rate
        return {'err': err, 'corr_hz': corr, 'cic': cic,
                'times': times, 'sample_rate': rate}
