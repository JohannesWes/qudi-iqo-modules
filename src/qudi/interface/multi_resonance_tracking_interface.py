# -*- coding: utf-8 -*-
"""
Interface for multi-resonance ODMR frequency-tracking hardware.

Extends the single-resonance frequency-lock concept (OdmrFreqLockInterface) to N
resonances tracked sequentially by hardware-driven LO hopping. The FPGA owns:
  - a per-channel phase-continuous modulation oscillator + freeze/settle window
    (pyrpl ``odmrmultitrack``, region 9),
  - N per-slot frequency-lock integrators selected by the live hop index
    (pyrpl ``odmrfreqlock``, region 8),
  - a continuous/loop hop generator + push stream + hop-boundary markers
    (pyrpl ``scan``, region 5).

The per-resonance SSB calibration and the LO jump-list live on the microwave/IF
source side (it owns the calibration tables and the LO), so they are NOT part of
this interface.

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

from abc import abstractmethod
from typing import Dict, Any, List, Optional, Tuple
from qudi.core.module import Base


class MultiResonanceTrackingInterface(Base):
    """
    Interface for hardware-based multi-resonance ODMR frequency tracking.

    N resonances share one LO and one IQ mixer; the LO hops between them under a
    continuous on-FPGA loop, and each resonance has its own freeze-and-resume
    demod chain + integrator. Loop gains, deadband and saturation limit are GLOBAL
    (shared by all slots in the current hardware); only the per-slot SSB cal (on
    the microwave side) and the per-slot integrator state differ.
    """

    @property
    @abstractmethod
    def nslots(self) -> int:
        """Number of resonance slots N implemented in hardware."""
        pass

    @property
    @abstractmethod
    def tracking_engine_active(self) -> bool:
        """Whether the continuous acquisition/hop engine is running.

        This is deliberately independent of the FPGA lock enable state.  For N=1
        the implementation may use a stationary slot rather than an actual hop FSM.
        """
        pass

    @property
    @abstractmethod
    def trace_stream_active(self) -> bool:
        """Whether the simultaneous error/correction push stream is running."""
        pass

    @property
    @abstractmethod
    def position_markers_active(self) -> bool:
        """Whether a motor scan currently owns the trace stream and its markers."""
        pass

    # =========================================================================
    # Modulation oscillator + freeze/settle (odmrmultitrack)
    # =========================================================================

    @abstractmethod
    def configure_oscillator(self, f_m_hz: float, demod_phase_deg: float,
                             settle_time_s: float, source: str = 'current_step') -> None:
        """
        Configure the per-channel modulation oscillator and freeze window.

        Args:
            f_m_hz: Shared modulation frequency f_m (Hz, e.g. 15250.0).
            demod_phase_deg: Demodulation phase 2*pi*f_m*tau (degrees).
            settle_time_s: Physical-settle hold-off after each hop (s). The chain
                is frozen during this window so the LO transient + pipeline tail
                are gated out (typical 100-300 us).
            source: 'current_step' (hardware hop index drives channel selection,
                the normal tracking mode) or 'sw' (software override, bring-up).
        """
        pass

    @abstractmethod
    def enable_oscillator(self, enable: bool) -> None:
        """Enable/disable the multi-channel oscillator + freeze gating. When
        disabled the hardware reverts to the legacy single-resonance signal path."""
        pass

    # =========================================================================
    # Per-slot frequency-lock integrators (odmrfreqlock)
    # =========================================================================

    @abstractmethod
    def set_bandwidth(self, bandwidth_hz: float, slope_lsb_per_hz: float,
                      pi: bool = False, zero_ratio: float = 3.0) -> None:
        """
        Configure the (global) loop gains.

        Gains are shared across all slots in the current hardware, so a single
        representative slope sets the integral/proportional gains; the effective
        per-resonance bandwidth then scales with that resonance's own slope.

        Args:
            bandwidth_hz: Target closed-loop bandwidth (Hz).
            slope_lsb_per_hz: Representative discriminator slope (LSB/Hz, > 0).
            pi: If True, configure PI mode (faster acquisition); else integral-only.
            zero_ratio: PI zero placement ratio alpha (PI mode only).
        """
        pass

    @abstractmethod
    def set_integrator_source(self, source: str = 'current_step') -> None:
        """Select which slot integrates: 'current_step' (hardware hop index, the
        tracking mode) or 'sw' (software-selected active slot, bring-up)."""
        pass

    @abstractmethod
    def enable_lock(self, enable: bool) -> None:
        """Enable/disable the frequency-lock loop (all slots)."""
        pass

    @abstractmethod
    def clear_integrators(self) -> None:
        """Clear all per-slot integrator states (reset corrections to zero)."""
        pass

    @abstractmethod
    def set_max_correction_hz(self, max_correction_hz: float) -> None:
        """Set the (global) integrator saturation limit +/-max_correction_hz."""
        pass

    @abstractmethod
    def set_invert(self, inverted: bool) -> None:
        """Set error-signal polarity (True for LSB, False for USB; see
        OdmrFreqLockInterface.set_invert)."""
        pass

    @abstractmethod
    def get_slot_status(self, slot: int) -> Dict[str, Any]:
        """
        Per-slot lock status.

        Returns:
            dict with at least: enabled (bool), locked (bool), saturated (bool),
            error_lsb (float), correction_hz (float).
        """
        pass

    @abstractmethod
    def get_all_status(self) -> List[Dict[str, Any]]:
        """Per-slot status for all slots (list indexed by slot)."""
        pass

    # =========================================================================
    # Continuous hardware hopping + optional per-resonance trace stream (scan)
    # =========================================================================

    @abstractmethod
    def start_tracking(self, nslots: int, dwell_time_s: float,
                       settling_time_s: float, trigger_length_s: float,
                       stream_traces: bool = True) -> None:
        """
        Start indefinite hardware-driven LO hopping over ``nslots`` resonances.

        For ``nslots > 1`` the scan FSM runs in continuous/loop mode: it emits one
        LO-hop trigger per slot and advances the hop index 0..nslots-1 forever (no
        software re-arming) until :meth:`stop_tracking`. For ``nslots == 1`` the
        implementation may keep software slot 0 selected and avoid starting the hop
        FSM. Per-slot corrections are then live-readable via :meth:`get_slot_status`.

        Args:
            nslots: number of resonances N (== LO jump-list length).
            dwell_time_s: per-resonance live time (sets the hop cadence for
                ``nslots > 1``; ignored by single-slot no-hop implementations).
            settling_time_s: LO-settle delay before the dwell (s), for
                ``nslots > 1`` hop timing.
            trigger_length_s: LO-hop trigger pulse length (s), for
                ``nslots > 1`` hop timing.
            stream_traces: if True, also start the push stream + hop markers so
                :meth:`read_traces` returns the high-rate per-resonance time traces.
        """
        pass

    @abstractmethod
    def stop_tracking(self) -> None:
        """Stop continuous hopping (and the trace stream if running)."""
        pass

    @abstractmethod
    def start_trace_stream(self) -> None:
        """Start the simultaneous error/correction stream without changing the
        running hop FSM or the FPGA lock enable state.

        The hopping/acquisition engine must already have been started with
        :meth:`start_tracking`.
        """
        pass

    @abstractmethod
    def stop_trace_stream(self) -> None:
        """Stop only the push stream, leaving hopping and the FPGA lock untouched."""
        pass

    @abstractmethod
    def set_trace_window_seconds(self, seconds: float) -> None:
        """
        Set the duration (s) of the high-rate rolling display window returned by
        :meth:`read_traces`. Live-settable; shorter windows give finer displayed
        resolution (fixed display-point budget), longer windows show more history.
        Does not affect the low-rate per-slot drift history.
        """
        pass

    @abstractmethod
    def read_traces(self) -> Optional[Dict[str, Any]]:
        """
        Reconstructed per-resonance time traces since session start (high-rate).

        Decodes the dual-quantity self-describing stream into four simultaneous
        per-resonance traces (both errors AND both corrections). Only meaningful
        when started with ``stream_traces=True``.

        Returns:
            None if not streaming, else a dict with:
              - 'times' (np.ndarray): common elapsed-since-stream-start time axis
                (s) for the returned window. A rolling window must retain its
                absolute session offset rather than restarting at zero after trim.
              - 'err' (np.ndarray, shape (nslots, len(times))): each row is a
                resonance's error signal (raw LSB) on the common axis (fresh while
                live, zero-order-hold while parked; NaN = transport loss).
              - 'corr_hz' (np.ndarray, shape (nslots, len(times))): each row is a
                resonance's frequency correction (Hz), same axis/semantics.
              - 'sample_rate' (float): stream sample rate (Hz), per-sample (triplet).
        """
        pass

    # =========================================================================
    # 2D motor-scan composition: x/y position markers on the running stream
    # =========================================================================
    # The tracker's continuous MARKED stream carries the resonance label inline in
    # the demod ring, so the FPGA's x/y marker banks are free. A 2D motor scan can
    # therefore add KDC encoder-position markers on top of the SAME logical stream,
    # then spatially bin the reconstructed per-resonance traces. Implementations may
    # restart/reset only the push-stream transport to establish an exact marker origin;
    # the hop engine and lock integrators must remain undisturbed. While a mapped scan
    # owns the physical stream drain, the caller must be the SOLE consumer of
    # read_stream_words(); read_traces() must not also drain.

    @abstractmethod
    def enable_position_markers(self, enable: bool) -> None:
        """
        Enable/disable KDC x/y position-marker capture on the running trace stream.

        Requires an active trace stream (the FPGA lock itself may be on or off).
        Enabling records
        the fast-axis (bin-boundary) and slow-axis (line-boundary) encoder pulses
        into the otherwise-free FPGA marker banks WITHOUT disturbing or resetting the
        per-resonance triplet stream. Correct per-bin, fresh-only binning needs the
        MARKED stream (uniform grid, no zero-order hold); implementations should warn
        if a different stream source is active. Implementations must raise if enabling
        is requested without an active stream, so a motor scan cannot silently acquire
        empty maps.
        """
        pass

    @abstractmethod
    def read_position_markers(self) -> Tuple[Any, Any]:
        """
        New (x, y) position markers since the last call, as TRIPLET (time-sample)
        indices into the reconstructed per-resonance traces.

        The raw FPGA markers are WORD indices (3 words per [err, corr, step]
        triplet); this returns them already converted to triplet indices, so they
        index directly into the ``read_stream_words``/``reconstruct_mapped_traces``
        sample axis. x = fast-axis bin boundaries, y = slow-axis line boundaries.

        Returns:
            (x, y): two int64 np.ndarrays (empty if markers disabled / none new).
        """
        pass

    @abstractmethod
    def read_stream_words(self) -> Any:
        """
        New raw triplet stream words since the last call (float64, NaN = transport
        loss), for a mapped 2D scan that reconstructs + spatially bins the traces.

        The caller accumulates these into one contiguous array (index 0 = first word
        of the session, aligned with the position markers) and decodes it with
        :meth:`reconstruct_mapped_traces`. This is the destructive stream drain: while
        a mapped scan is running the caller must be the only consumer. Empty if not
        streaming.
        """
        pass

    @abstractmethod
    def reconstruct_mapped_traces(self, words: Any) -> Dict[str, Any]:
        """
        Decode an accumulated raw triplet-word array into per-resonance err/corr
        traces on the uniform sample grid, FRESH-ONLY (NaN when parked / in the
        per-hop dead-time / transport loss -- no zero-order hold), so a per-bin
        ``nanmean`` sees only that resonance's own live samples.

        Args:
            words: contiguous triplet-word array (float64, NaN = loss), index 0 =
                first word of the session.

        Returns:
            dict with 'err' (N, T) raw LSB, 'corr_hz' (N, T) Hz, 'times' (T,) s, and
            'sample_rate' (float, Hz), where T is the number of complete triplets.
        """
        pass
