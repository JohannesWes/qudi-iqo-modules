# -*- coding: utf-8 -*-
"""
Dummy ODMR Logic for testing motor scan functionality.

Simulates ODMR scan behavior with synthetic hyperfine spectra.

Copyright (c) 2024, the qudi developers.

Example config:

    odmr_logic_dummy:
        module.Class: 'dummy.odmr_logic_dummy.OdmrLogicDummy'
        options:
            center_frequency: 2.87e9    # GHz
            hyperfine_splitting: 2.2e6  # Hz
            scan_duration: 1.0          # seconds
"""

import time
import numpy as np
from typing import Dict, List, Optional

from PySide2 import QtCore

from qudi.core.module import LogicBase
from qudi.core.configoption import ConfigOption


class OdmrLogicDummy(LogicBase):
    """
    Dummy ODMR Logic for testing motor scan functionality.
    
    Generates synthetic ODMR spectra with hyperfine structure
    to test the motor scan logic and GUI.
    """
    
    # Config options
    _center_frequency = ConfigOption(name='center_frequency', default=2.87e9)
    _hyperfine_splitting = ConfigOption(name='hyperfine_splitting', default=2.2e6)
    _scan_duration = ConfigOption(name='scan_duration', default=1.0)
    _linewidth = ConfigOption(name='linewidth', default=0.1e6)
    
    # Signals matching OdmrLogic interface
    sigScanStateUpdated = QtCore.Signal(bool)
    sigScanDataUpdated = QtCore.Signal()
    sigElapsedUpdated = QtCore.Signal(float, int)
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self._is_running = False
        self._frequency_data = []
        self._signal_data = {}
        self._raw_data = {}
        self._elapsed_sweeps = 0
        
        # Scan parameters
        self._scan_frequency_ranges = [(2.85e9, 2.89e9, 201)]
        self._scan_power = -10.0
        
        # Position-dependent variation
        self._current_position = {'x': 0.0, 'y': 0.0}
    
    def on_activate(self):
        """Initialize the module."""
        self._initialize_odmr_data()
        self.log.info("OdmrLogicDummy activated.")
    
    def on_deactivate(self):
        """Clean up."""
        self.log.info("OdmrLogicDummy deactivated.")
    
    def _initialize_odmr_data(self):
        """Initialize data arrays."""
        self._frequency_data = []
        for start, stop, points in self._scan_frequency_ranges:
            self._frequency_data.append(np.linspace(start, stop, int(points)))
        
        self._signal_data = {'Voltage': [np.zeros(len(f)) for f in self._frequency_data]}
        self._raw_data = {'Voltage': [np.zeros((len(f), 100)) for f in self._frequency_data]}
    
    @property
    def frequency_data(self) -> List:
        """Return frequency data."""
        return self._frequency_data.copy()
    
    @property
    def signal_data(self) -> Dict:
        """Return signal data."""
        return {k: [arr.copy() for arr in v] for k, v in self._signal_data.items()}
    
    @property
    def frequency_ranges(self):
        """Return scan frequency ranges."""
        return self._scan_frequency_ranges.copy()
    
    def set_position_hint(self, position: Dict[str, float]):
        """
        Set position hint for position-dependent ODMR simulation.
        
        Args:
            position: Current motor position dict.
        """
        self._current_position = position.copy()
    
    def start_odmr_scan(self):
        """Start a simulated ODMR scan."""
        if self._is_running:
            self.log.warning("ODMR scan already running.")
            return
        
        self._is_running = True
        self.module_state.lock()
        self.sigScanStateUpdated.emit(True)
        
        # Simulate scan in background
        QtCore.QTimer.singleShot(
            int(self._scan_duration * 1000),
            self._complete_scan
        )
    
    def stop_odmr_scan(self):
        """Stop the ODMR scan."""
        if self._is_running:
            self._is_running = False
            if self.module_state() == 'locked':
                self.module_state.unlock()
            self.sigScanStateUpdated.emit(False)
    
    def _complete_scan(self):
        """Complete the simulated scan with synthetic data."""
        if not self._is_running:
            return
        
        # Generate synthetic hyperfine spectrum
        self._generate_synthetic_odmr()
        
        self._elapsed_sweeps += 1
        self._is_running = False
        
        if self.module_state() == 'locked':
            self.module_state.unlock()
        
        self.sigScanDataUpdated.emit()
        self.sigScanStateUpdated.emit(False)
    
    def _generate_synthetic_odmr(self):
        """
        Generate synthetic ODMR spectrum with hyperfine structure.
        
        Creates 3 peaks (hyperfine lines) with position-dependent variation.
        """
        for range_idx, freq_array in enumerate(self._frequency_data):
            # Base parameters
            center = self._center_frequency
            splitting = self._hyperfine_splitting
            linewidth = self._linewidth
            
            # Add position-dependent variation
            x_pos = self._current_position.get('x', 0) * 1000  # mm
            y_pos = self._current_position.get('y', 0) * 1000  # mm
            
            # Frequency shift with position (simulates magnetic gradient)
            freq_shift = (x_pos * 50e3 + y_pos * 30e3)  # Hz per mm
            center += freq_shift
            
            # Signal contrast variation with position
            contrast_variation = 1.0 + 0.1 * np.sin(x_pos * 0.5) * np.cos(y_pos * 0.5)
            
            # Create the spectrum with 3 hyperfine peaks
            signal = np.ones_like(freq_array)
            
            for i, offset in enumerate([-splitting, 0, splitting]):
                peak_center = center + offset
                
                # Lorentzian dip
                dip = (linewidth / 2)**2 / ((freq_array - peak_center)**2 + (linewidth / 2)**2)
                signal -= 0.02 * contrast_variation * dip
            
            # Add noise
            noise = np.random.normal(0, 0.002, len(freq_array))
            signal += noise
            
            self._signal_data['Voltage'][range_idx] = signal
