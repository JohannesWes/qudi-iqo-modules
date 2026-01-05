# -*- coding: utf-8 -*-
"""
Motor scan logic package for qudi.

This package provides motor-based XY scanning functionality with support for
multiple data acquisition modes (STEP_ODMR, CONTINUOUS_STREAM, CONTINUOUS_FREQ_TRACK).

The package is organized into modules for better maintainability:
- data_structures: Enums and MotorScanData dataclass
- motor_control: Motor movement and homing mixin
- data_processing: Data acquisition and processing mixin  
- data_saving: Data saving and figure generation mixin
- scan_logic: Main MotorScanLogic class

For backward compatibility, all public symbols are re-exported from this module,
so existing code using `from qudi.logic.motor_scan_logic import ...` will continue
to work after updating the import to `from qudi.logic.motor_scan import ...`.

Copyright (c) 2024, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>
"""

# Re-export all public symbols for backward compatibility
from .data_structures import (
    ScanMode,
    ScanPattern,
    ScanState,
    MotorScanData,
)

from .scan_logic import MotorScanLogic

# Define what gets exported with "from motor_scan import *"
__all__ = [
    'ScanMode',
    'ScanPattern', 
    'ScanState',
    'MotorScanData',
    'MotorScanLogic',
]
