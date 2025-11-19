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

from typing import Dict, Any
from qudi.core.configoption import ConfigOption
from qudi.interface.odmr_freq_lock_interface import OdmrFreqLockInterface
from qudi.hardware.redpitaya.resource_manager import get_pyrpl_instance, release_pyrpl_instance


class RedPitayaOdmrLockHardware(OdmrFreqLockInterface):
    """
    Hardware interface to Red Pitaya ODMR frequency lock via PyRPL.

    Config example:
        redpitaya_odmr_lock:
            module.Class: 'redpitaya.redpitaya_odmr_lock.RedPitayaOdmrLockHardware'
            options:
                redpitaya_config_name: 'rpy_shared_config'
                redpitaya_hostname: '10.203.129.28'
    """

    _redpitaya_config_name = ConfigOption('redpitaya_config_name',
                                          default='rpy_shared_config', missing='info')
    _redpitaya_hostname = ConfigOption('redpitaya_hostname', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pyrpl = None
        self._lock = None

    def on_activate(self):
        """Connect to PyRPL and get odmr_freq_lock module."""
        # Get shared PyRPL instance via resource manager
        self._pyrpl, _ = get_pyrpl_instance(
            hostname=self._redpitaya_hostname,
            config_name=self._redpitaya_config_name
        )

        # Get odmrfreqlock module (PyRPL naming: all lowercase, no underscores)
        self._lock = self._pyrpl.rp.odmrfreqlock

        # Ensure lock is disabled on activation
        self._lock.enable = False

        self.log.info(f'Red Pitaya ODMR Lock connected: {self._redpitaya_hostname}')

    def on_deactivate(self):
        """Disable lock and disconnect."""
        if self._lock is not None:
            try:
                self._lock.enable = False
            except Exception as e:
                self.log.warning(f'Could not disable lock on deactivation: {e}')

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

    def set_bandwidth(self, bandwidth_hz: float, slope_lsb_per_hz: float) -> None:
        """Configure integral-only frequency lock."""
        if bandwidth_hz <= 0:
            raise ValueError(f'Bandwidth must be positive, got {bandwidth_hz}')
        if slope_lsb_per_hz <= 0:
            raise ValueError(f'Slope must be positive, got {slope_lsb_per_hz}')

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
        if not 2.0 <= zero_ratio <= 4.0:
            raise ValueError(f'Zero ratio must be in [2.0, 4.0], got {zero_ratio}')

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
