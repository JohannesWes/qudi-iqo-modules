# -*- coding: utf-8 -*-
"""
Motor-based XY scanning GUI module for qudi.

Provides visualization and control for motor-based XY scanning with
ODMR or continuous streaming data acquisition.

Copyright (c) 2024, the qudi developers.

Example config:

    motor_scan_gui:
        module.Class: 'motor_scan.motor_scan_gui.MotorScanGui'
        connect:
            motor_scan_logic: motor_scan_logic
"""

import os
import numpy as np
from typing import Optional, Dict, Tuple
from PySide2 import QtCore, QtWidgets, QtGui

from qudi.core.module import GuiBase
from qudi.core.connector import Connector
from qudi.util.paths import get_artwork_dir
from qudi.util.colordefs import QudiPalettePale as palette
from qudi.util.widgets.plotting.image_widget import RubberbandZoomSelectionImageWidget
from qudi.gui.motor_scan.pixel_odmr_widget import PixelOdmrSpectrumWidget

# KDC_HW_SYNC_MULTIRES display selector: individual per-resonance correction/error
# plus the difference/sum of the two corrections. Maps a display label to how the
# 2D data array is built from the 4 stored channels (res{k}_err / res{k}_corr).
MULTIRES_DISPLAY_OPTIONS = [
    'Res0 Correction', 'Res1 Correction',
    'Res0 Error', 'Res1 Error',
    'Δ Correction (R1−R0)', 'Σ Correction (R0+R1)',
]


class MotorScanMainWindow(QtWidgets.QMainWindow):
    """Main window for Motor Scan GUI."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Motor XY Scan')
        self.setMinimumSize(800, 600)
        
        # Central widget
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        
        # Main layout
        self.main_layout = QtWidgets.QVBoxLayout(self.central_widget)
        
        # Create UI sections
        self._create_toolbar()
        self._create_scan_display()
        self._create_control_panel()
        self._create_stage_control_panel()
        self._create_status_bar()
    
    def _create_toolbar(self):
        """Create the main toolbar."""
        self.toolbar = QtWidgets.QToolBar('Main Toolbar')
        self.addToolBar(self.toolbar)
        
        # Start/Stop scan action
        icon_path = get_artwork_dir()
        
        self.action_start_scan = QtWidgets.QAction('Start Scan', self)
        self.action_start_scan.setCheckable(True)
        start_icon = QtGui.QIcon()
        start_icon.addFile(os.path.join(icon_path, 'icons', 'start-counter.svg'),
                          QtCore.QSize(), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        start_icon.addFile(os.path.join(icon_path, 'icons', 'stop-counter.svg'),
                          QtCore.QSize(), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.action_start_scan.setIcon(start_icon)
        self.toolbar.addAction(self.action_start_scan)
        
        # Pause action
        self.action_pause_scan = QtWidgets.QAction('Pause', self)
        self.action_pause_scan.setCheckable(True)
        self.action_pause_scan.setEnabled(False)
        self.toolbar.addAction(self.action_pause_scan)
        
        self.toolbar.addSeparator()
        
        # Home stages action
        self.action_home_stages = QtWidgets.QAction('Home Stages', self)
        self.action_home_stages.setToolTip(
            'Home (calibrate) the motor stages.\n'
            'Moves stages to home position and establishes zero reference.\n'
            'Cannot be used during a scan.'
        )
        self.toolbar.addAction(self.action_home_stages)
        
        self.toolbar.addSeparator()
        
        # Save action
        self.action_save = QtWidgets.QAction('Save Data', self)
        save_icon = QtGui.QIcon(os.path.join(icon_path, 'icons', 'document-save.svg'))
        self.action_save.setIcon(save_icon)
        self.action_save.setToolTip(
            'Save scan data.\n'
            'Use text field to specify a nametag for the file.'
        )
        self.toolbar.addAction(self.action_save)
        
        # Save nametag line edit
        self.save_nametag_lineedit = QtWidgets.QLineEdit()
        self.save_nametag_lineedit.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                                  QtWidgets.QSizePolicy.Fixed)
        self.save_nametag_lineedit.setMinimumWidth(150)
        self.save_nametag_lineedit.setPlaceholderText('Enter save tag...')
        self.save_nametag_lineedit.setToolTip('Enter a nametag to include in saved file name')
        self.toolbar.addWidget(self.save_nametag_lineedit)

        # Load action
        self.action_load = QtWidgets.QAction('Load Data', self)
        load_icon = QtGui.QIcon(os.path.join(icon_path, 'icons', 'document-open.svg'))
        self.action_load.setIcon(load_icon)
        self.action_load.setToolTip(
            'Load previously saved STEP_ODMR scan data.\n'
            'Allows reviewing data and viewing per-pixel ODMR spectra.\n'
            'Only available in STEP_ODMR mode when no scan is running.'
        )
        self.toolbar.addAction(self.action_load)

        self.toolbar.addSeparator()
        
        # Scan mode selector
        self.mode_label = QtWidgets.QLabel(' Mode: ')
        self.toolbar.addWidget(self.mode_label)
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(['STEP_ODMR', 'CONTINUOUS_STREAM', 'CONTINUOUS_FREQ_TRACK',
                                  'POSITION_ONLY', 'KDC_HW_SYNC', 'KDC_HW_SYNC_MULTIRES'])
        self.mode_combo.setToolTip(
            'STEP_ODMR: Stop at each point, take ODMR spectrum\n'
            'CONTINUOUS_STREAM: Continuous movement, stream channel data\n'
            'CONTINUOUS_FREQ_TRACK: Continuous movement, record absolute frequency from lock\n'
            'POSITION_ONLY: Move stage along pattern without data acquisition (debugging)\n'
            'KDC_HW_SYNC: Continuous movement; KDC position-step triggers bin the FPGA '
            'demod stream in hardware (exact sync, no line shifts)\n'
            'KDC_HW_SYNC_MULTIRES: As KDC_HW_SYNC, but bins BOTH resonances\' error + '
            'correction per pixel (4 maps). Configure and Start Stream in the Multi-'
            'Resonance ODMR GUI first; Start Tracking is optional (open-loop supported).'
        )
        self.toolbar.addWidget(self.mode_combo)
        
        # Scan pattern selector
        self.pattern_label = QtWidgets.QLabel(' Pattern: ')
        self.toolbar.addWidget(self.pattern_label)
        self.pattern_combo = QtWidgets.QComboBox()
        self.pattern_combo.addItems(['SNAKE_X', 'LINE_BY_LINE_X', 'SNAKE_Y', 'LINE_BY_LINE_Y'])
        self.pattern_combo.setToolTip(
            'SNAKE_X: Scan X lines, alternate direction (efficient)\n'
            'LINE_BY_LINE_X: Scan X lines, return to start each line\n'
            'SNAKE_Y: Scan Y lines, alternate direction\n'
            'LINE_BY_LINE_Y: Scan Y lines, return to start each line'
        )
        self.toolbar.addWidget(self.pattern_combo)

        # KDC_HW_SYNC: keep+save the full per-bin time-traces, or just the mean map.
        self.save_traces_check = QtWidgets.QCheckBox(' Save full traces ')
        self.save_traces_check.setToolTip(
            'KDC_HW_SYNC / KDC_HW_SYNC_MULTIRES: if checked, the full cut time-trace '
            'of every position bin (for MULTIRES, per resonance and quantity) is kept '
            'and saved. If unchecked (default), only the per-bin MEAN map is built and '
            'saved -- lower memory and disk. The mean map is always what is shown here.')
        self.toolbar.addWidget(self.save_traces_check)

        self.toolbar.addSeparator()

        # Display channel selector
        self.channel_label = QtWidgets.QLabel(' Display: ')
        self.toolbar.addWidget(self.channel_label)
        self.display_combo = QtWidgets.QComboBox()
        self.display_combo.addItems(['Center Frequency', 'Linewidth', 'Splitting', 'Fit Quality'])
        self.toolbar.addWidget(self.display_combo)
    
    def _create_scan_display(self):
        """Create the main scan image display with optional pixel spectrum viewer."""
        # Create splitter to hold scan image and spectrum viewer
        self.display_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # Scan image widget (left side)
        self.scan_groupbox = QtWidgets.QGroupBox('Scan Image')
        scan_layout = QtWidgets.QVBoxLayout(self.scan_groupbox)

        self.image_widget = RubberbandZoomSelectionImageWidget(
            allow_tracking_outside_data=True,
            xy_region_selection_crosshair=True,
            xy_region_selection_handles=False
        )
        self.image_widget.set_selection_mutable(True)
        self.image_widget.set_axis_label('bottom', label='X Position', unit='m')
        self.image_widget.set_axis_label('left', label='Y Position', unit='m')
        self.image_widget.set_data_label(label='Value', unit='')

        scan_layout.addWidget(self.image_widget)
        self.display_splitter.addWidget(self.scan_groupbox)

        # Pixel ODMR spectrum viewer (right side) - only for STEP_ODMR mode
        self.pixel_spectrum_groupbox = QtWidgets.QGroupBox('Pixel ODMR Spectrum')
        spectrum_layout = QtWidgets.QVBoxLayout(self.pixel_spectrum_groupbox)
        self.pixel_spectrum_widget = PixelOdmrSpectrumWidget()
        spectrum_layout.addWidget(self.pixel_spectrum_widget)
        self.display_splitter.addWidget(self.pixel_spectrum_groupbox)

        # Set initial sizes (60% scan image, 40% spectrum)
        self.display_splitter.setSizes([600, 400])

        # Hide spectrum panel initially (shown only in STEP_ODMR mode)
        self.pixel_spectrum_groupbox.setVisible(False)

        self.main_layout.addWidget(self.display_splitter, stretch=3)
    
    def _create_control_panel(self):
        """Create the scan control panel."""
        control_group = QtWidgets.QGroupBox('Scan Settings')
        control_layout = QtWidgets.QGridLayout(control_group)
        
        # X axis settings
        control_layout.addWidget(QtWidgets.QLabel('X Start (mm):'), 0, 0)
        self.x_start_spinbox = QtWidgets.QDoubleSpinBox()
        self.x_start_spinbox.setRange(0, 50)
        self.x_start_spinbox.setDecimals(3)
        self.x_start_spinbox.setValue(0)
        self.x_start_spinbox.setSuffix(' mm')
        control_layout.addWidget(self.x_start_spinbox, 0, 1)
        
        control_layout.addWidget(QtWidgets.QLabel('X Stop (mm):'), 0, 2)
        self.x_stop_spinbox = QtWidgets.QDoubleSpinBox()
        self.x_stop_spinbox.setRange(0, 50)
        self.x_stop_spinbox.setDecimals(3)
        self.x_stop_spinbox.setValue(10)
        self.x_stop_spinbox.setSuffix(' mm')
        control_layout.addWidget(self.x_stop_spinbox, 0, 3)
        
        control_layout.addWidget(QtWidgets.QLabel('X Points:'), 0, 4)
        self.x_points_spinbox = QtWidgets.QSpinBox()
        self.x_points_spinbox.setRange(1, 1000)
        self.x_points_spinbox.setValue(20)
        control_layout.addWidget(self.x_points_spinbox, 0, 5)
        
        # Y axis settings
        control_layout.addWidget(QtWidgets.QLabel('Y Start (mm):'), 1, 0)
        self.y_start_spinbox = QtWidgets.QDoubleSpinBox()
        self.y_start_spinbox.setRange(0, 50)
        self.y_start_spinbox.setDecimals(3)
        self.y_start_spinbox.setValue(0)
        self.y_start_spinbox.setSuffix(' mm')
        control_layout.addWidget(self.y_start_spinbox, 1, 1)
        
        control_layout.addWidget(QtWidgets.QLabel('Y Stop (mm):'), 1, 2)
        self.y_stop_spinbox = QtWidgets.QDoubleSpinBox()
        self.y_stop_spinbox.setRange(0, 50)
        self.y_stop_spinbox.setDecimals(3)
        self.y_stop_spinbox.setValue(10)
        self.y_stop_spinbox.setSuffix(' mm')
        control_layout.addWidget(self.y_stop_spinbox, 1, 3)
        
        control_layout.addWidget(QtWidgets.QLabel('Y Points:'), 1, 4)
        self.y_points_spinbox = QtWidgets.QSpinBox()
        self.y_points_spinbox.setRange(1, 1000)
        self.y_points_spinbox.setValue(20)
        control_layout.addWidget(self.y_points_spinbox, 1, 5)
        
        # Apply button
        self.apply_settings_button = QtWidgets.QPushButton('Apply Settings')
        control_layout.addWidget(self.apply_settings_button, 2, 4, 1, 2)
        
        # Progress bar
        control_layout.addWidget(QtWidgets.QLabel('Progress:'), 2, 0)
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        control_layout.addWidget(self.progress_bar, 2, 1, 1, 3)
        
        self.main_layout.addWidget(control_group, stretch=1)

    def _create_stage_control_panel(self):
        """Create the stage control panel for manual positioning."""
        stage_group = QtWidgets.QGroupBox('Stage Control')
        stage_layout = QtWidgets.QGridLayout(stage_group)

        # Target X position
        stage_layout.addWidget(QtWidgets.QLabel('Target X (mm):'), 0, 0)
        self.move_x_spinbox = QtWidgets.QDoubleSpinBox()
        self.move_x_spinbox.setRange(0, 50)
        self.move_x_spinbox.setDecimals(3)
        self.move_x_spinbox.setValue(0)
        self.move_x_spinbox.setSuffix(' mm')
        stage_layout.addWidget(self.move_x_spinbox, 0, 1)

        # Target Y position
        stage_layout.addWidget(QtWidgets.QLabel('Target Y (mm):'), 0, 2)
        self.move_y_spinbox = QtWidgets.QDoubleSpinBox()
        self.move_y_spinbox.setRange(0, 50)
        self.move_y_spinbox.setDecimals(3)
        self.move_y_spinbox.setValue(0)
        self.move_y_spinbox.setSuffix(' mm')
        stage_layout.addWidget(self.move_y_spinbox, 0, 3)

        # Buttons row
        self.move_to_start_button = QtWidgets.QPushButton('Go to Scan Start')
        self.move_to_start_button.setToolTip('Move stages to the scan start position (X Start, Y Start)')
        stage_layout.addWidget(self.move_to_start_button, 1, 0, 1, 2)

        self.move_button = QtWidgets.QPushButton('Move to Position')
        self.move_button.setToolTip('Move stages to the target position specified above')
        stage_layout.addWidget(self.move_button, 1, 2)

        self.stop_move_button = QtWidgets.QPushButton('Stop')
        self.stop_move_button.setToolTip('Stop current movement')
        self.stop_move_button.setEnabled(False)  # Disabled until movement starts
        stage_layout.addWidget(self.stop_move_button, 1, 3)

        self.main_layout.addWidget(stage_group, stretch=0)

    def _create_status_bar(self):
        """Create the status bar."""
        self.statusbar = QtWidgets.QStatusBar()
        self.setStatusBar(self.statusbar)

        # Position display
        self.position_label = QtWidgets.QLabel('Position: X=0.000mm, Y=0.000mm')
        self.statusbar.addPermanentWidget(self.position_label)

        # Lock status indicator (for CONTINUOUS_FREQ_TRACK mode)
        self.lock_status_label = QtWidgets.QLabel('')
        self.lock_status_label.setVisible(False)  # Hidden by default
        self.statusbar.addPermanentWidget(self.lock_status_label)

        # Loaded data indicator
        self.loaded_data_label = QtWidgets.QLabel('')
        self.loaded_data_label.setStyleSheet('color: #CC8800; font-weight: bold; padding: 0 10px;')
        self.loaded_data_label.setVisible(False)  # Hidden by default
        self.statusbar.addPermanentWidget(self.loaded_data_label)

        # Scan status
        self.scan_status_label = QtWidgets.QLabel('Idle')
        self.statusbar.addWidget(self.scan_status_label)


class MotorScanGui(GuiBase):
    """
    GUI module for motor-based XY scanning.
    
    Provides visualization of scan data and control over scan parameters,
    supporting both STEP_ODMR and CONTINUOUS_STREAM modes.
    
    Example config:
    
        motor_scan_gui:
            module.Class: 'motor_scan.motor_scan_gui.MotorScanGui'
            connect:
                motor_scan_logic: motor_scan_logic
    """
    
    # Connectors
    _motor_scan_logic = Connector(interface='LogicBase', name='motor_scan_logic')
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mw = None
        self._selected_pixel = None  # Tuple (ix, iy) or None for pixel spectrum viewer
    
    def on_activate(self):
        """Initialize the GUI."""
        # Create main window
        self._mw = MotorScanMainWindow()
        
        # Get logic reference
        self._logic = self._motor_scan_logic()
        
        # Connect signals from logic
        self._logic.sigScanStateChanged.connect(self._on_scan_state_changed)
        self._logic.sigScanDataUpdated.connect(self._on_scan_data_updated)
        self._logic.sigPositionUpdated.connect(self._on_position_updated)
        self._logic.sigScanSettingsChanged.connect(self._on_settings_changed)
        self._logic.sigScanCompleted.connect(self._on_scan_completed)
        self._logic.sigSaveStateChanged.connect(self._on_save_state_changed)
        self._logic.sigHomingStateChanged.connect(self._on_homing_state_changed)
        self._logic.sigMovementStateChanged.connect(self._on_movement_state_changed)
        self._logic.sigLockLostDuringScan.connect(self._on_lock_lost)
        self._logic.sigLockStatusUpdated.connect(self._on_lock_status_updated)
        self._logic.sigLoadedDataChanged.connect(self._on_loaded_data_changed)
        self._logic.sigScanStatusMessage.connect(self._on_scan_status_message)

        # Connect GUI signals
        self._mw.action_start_scan.triggered.connect(self._toggle_scan)
        self._mw.action_pause_scan.triggered.connect(self._toggle_pause)
        self._mw.action_save.triggered.connect(self._save_data)
        self._mw.action_load.triggered.connect(self._load_data)
        self._mw.action_home_stages.triggered.connect(self._home_stages)
        self._mw.apply_settings_button.clicked.connect(self._apply_settings)
        self._mw.mode_combo.currentTextChanged.connect(self._mode_changed)
        self._mw.pattern_combo.currentTextChanged.connect(self._pattern_changed)
        self._mw.display_combo.currentTextChanged.connect(self._display_channel_changed)
        self._mw.save_traces_check.toggled.connect(self._save_traces_toggled)

        # Connect stage control signals
        self._mw.move_button.clicked.connect(self._move_to_position)
        self._mw.move_to_start_button.clicked.connect(self._move_to_scan_start)
        self._mw.stop_move_button.clicked.connect(self._logic.stop_movement)

        # Connect pixel click signal for ODMR spectrum viewer
        # Connect to pyqtgraph scene's sigMouseClicked for reliable click detection
        self._mw.image_widget.plot_widget.scene().sigMouseClicked.connect(
            self._on_scene_mouse_clicked
        )

        # Initialize display from logic
        self._restore_settings_from_logic()
        
        self.show()
        self.log.info('MotorScanGui activated.')
    
    def on_deactivate(self):
        """Clean up on deactivation."""
        # Disconnect signals
        self._logic.sigScanStateChanged.disconnect(self._on_scan_state_changed)
        self._logic.sigScanDataUpdated.disconnect(self._on_scan_data_updated)
        self._logic.sigPositionUpdated.disconnect(self._on_position_updated)
        self._logic.sigScanSettingsChanged.disconnect(self._on_settings_changed)
        self._logic.sigScanCompleted.disconnect(self._on_scan_completed)
        self._logic.sigSaveStateChanged.disconnect(self._on_save_state_changed)
        self._logic.sigHomingStateChanged.disconnect(self._on_homing_state_changed)
        self._logic.sigMovementStateChanged.disconnect(self._on_movement_state_changed)
        self._logic.sigLockLostDuringScan.disconnect(self._on_lock_lost)
        self._logic.sigLockStatusUpdated.disconnect(self._on_lock_status_updated)
        self._logic.sigLoadedDataChanged.disconnect(self._on_loaded_data_changed)
        self._logic.sigScanStatusMessage.disconnect(self._on_scan_status_message)

        # Disconnect pixel click signal
        self._mw.image_widget.plot_widget.scene().sigMouseClicked.disconnect(
            self._on_scene_mouse_clicked
        )

        # Close window
        if self._mw is not None:
            self._mw.close()
        
        self.log.info('MotorScanGui deactivated.')
    
    def show(self):
        """Show the main window."""
        if self._mw is not None:
            self._mw.show()
            self._mw.raise_()
            self._mw.activateWindow()
    
    def _restore_settings_from_logic(self):
        """Restore GUI settings from logic module."""
        settings = self._logic.scan_settings
        
        # Update scan ranges
        if 'scan_ranges' in settings:
            ranges = settings['scan_ranges']
            if 'x' in ranges:
                self._mw.x_start_spinbox.setValue(ranges['x'][0] * 1000)  # m to mm
                self._mw.x_stop_spinbox.setValue(ranges['x'][1] * 1000)
            if 'y' in ranges:
                self._mw.y_start_spinbox.setValue(ranges['y'][0] * 1000)
                self._mw.y_stop_spinbox.setValue(ranges['y'][1] * 1000)
        
        # Update resolutions
        if 'scan_resolution' in settings:
            res = settings['scan_resolution']
            if 'x' in res:
                self._mw.x_points_spinbox.setValue(res['x'])
            if 'y' in res:
                self._mw.y_points_spinbox.setValue(res['y'])
        
        # Update mode
        if 'scan_mode' in settings:
            mode_val = settings['scan_mode']
            if isinstance(mode_val, int):
                from qudi.logic.motor_scan import ScanMode
                mode_name = ScanMode(mode_val).name
            else:
                mode_name = str(mode_val)
            index = self._mw.mode_combo.findText(mode_name)
            if index >= 0:
                self._mw.mode_combo.setCurrentIndex(index)
        
        # Update pattern
        if 'scan_pattern' in settings:
            pattern_val = settings['scan_pattern']
            if isinstance(pattern_val, str):
                pattern_name = pattern_val
            else:
                from qudi.logic.motor_scan import ScanPattern
                pattern_name = ScanPattern(pattern_val).name
            index = self._mw.pattern_combo.findText(pattern_name)
            if index >= 0:
                self._mw.pattern_combo.setCurrentIndex(index)

        # Restore the KDC_HW_SYNC "save full traces" toggle from the logic StatusVar
        try:
            self._mw.save_traces_check.blockSignals(True)
            self._mw.save_traces_check.setChecked(bool(self._logic.save_full_traces))
        finally:
            self._mw.save_traces_check.blockSignals(False)

        # Ensure pixel spectrum panel visibility matches current mode
        current_mode = self._mw.mode_combo.currentText()
        self._mw.pixel_spectrum_groupbox.setVisible(current_mode == 'STEP_ODMR')

        # Ensure Load button state matches current mode
        from qudi.logic.motor_scan import ScanState
        is_step_odmr = current_mode == 'STEP_ODMR'
        is_idle = self._logic.scan_state == ScanState.IDLE
        self._mw.action_load.setEnabled(is_step_odmr and is_idle)

    def _apply_settings(self):
        """Apply current GUI settings to logic."""
        # Block settings changed signal to avoid restoring values while we're updating
        self._logic.sigScanSettingsChanged.disconnect(self._on_settings_changed)
        
        try:
            # Convert mm to m
            x_start = self._mw.x_start_spinbox.value() / 1000
            x_stop = self._mw.x_stop_spinbox.value() / 1000
            y_start = self._mw.y_start_spinbox.value() / 1000
            y_stop = self._mw.y_stop_spinbox.value() / 1000
            
            self._logic.set_scan_ranges({
                'x': (x_start, x_stop),
                'y': (y_start, y_stop)
            })
            
            self._logic.set_scan_resolution({
                'x': self._mw.x_points_spinbox.value(),
                'y': self._mw.y_points_spinbox.value()
            })
        finally:
            # Reconnect signal
            self._logic.sigScanSettingsChanged.connect(self._on_settings_changed)
    
    def _toggle_scan(self, checked: bool):
        """Start or stop the scan."""
        if checked:
            self._apply_settings()
            self._logic.start_scan()
        else:
            self._logic.stop_scan()
    
    def _toggle_pause(self, checked: bool):
        """Pause or resume the scan."""
        if checked:
            self._logic.pause_scan()
        else:
            self._logic.resume_scan()
    
    def _save_data(self):
        """Save scan data with optional nametag."""
        nametag = self._mw.save_nametag_lineedit.text()
        self._logic.save_scan_data(nametag if nametag else None)
    
    def _home_stages(self):
        """Home (calibrate) the motor stages."""
        # Confirm with user since homing takes time
        reply = QtWidgets.QMessageBox.question(
            self._mw,
            'Home Stages',
            'This will home (calibrate) all motor stages.\n'
            'The stages will move to their home position.\n\n'
            'Continue?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        
        if reply == QtWidgets.QMessageBox.Yes:
            self._mw.scan_status_label.setText('Homing...')
            self._mw.action_home_stages.setEnabled(False)
            # Call home_stages which now runs asynchronously on the logic thread
            self._logic.home_stages()

    def _load_data(self):
        """Load previously saved scan data for review."""
        # Check if current mode is STEP_ODMR
        if self._mw.mode_combo.currentText() != 'STEP_ODMR':
            QtWidgets.QMessageBox.warning(
                self._mw,
                'Load Data',
                'Loading data is only supported in STEP_ODMR mode.\n\n'
                'Please switch to STEP_ODMR mode first.',
                QtWidgets.QMessageBox.Ok
            )
            return

        # Show confirmation dialog
        reply = QtWidgets.QMessageBox.question(
            self._mw,
            'Load Measurement Data',
            'Loading will replace the current scan view.\n\n'
            'If you have unsaved measurement data, you can still\n'
            'save it later (the data is preserved in the logic module).\n\n'
            'Do you want to continue?',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )

        if reply != QtWidgets.QMessageBox.Yes:
            return

        # Get default directory for file dialog
        default_dir = self._logic.module_default_data_dir

        # Show folder selection dialog
        folder = QtWidgets.QFileDialog.getExistingDirectory(
            self._mw,
            'Select Motor Scan Data Folder',
            default_dir,
            QtWidgets.QFileDialog.ShowDirsOnly
        )

        if folder:
            # Load the data through logic module
            success = self._logic.load_scan_data_from_folder(folder)
            if not success:
                QtWidgets.QMessageBox.warning(
                    self._mw,
                    'Load Failed',
                    'Failed to load scan data from the selected folder.\n\n'
                    'Please check the log for details.\n'
                    'Ensure the folder contains valid STEP_ODMR scan data.',
                    QtWidgets.QMessageBox.Ok
                )

    @QtCore.Slot(bool)
    def _on_loaded_data_changed(self, is_loaded: bool):
        """Handle loaded data state change from logic."""
        # Update loaded data indicator
        self._mw.loaded_data_label.setVisible(is_loaded)

        if is_loaded:
            self._mw.loaded_data_label.setText('[VIEWING LOADED DATA]')

            # Update window title with timestamp
            loaded_data = self._logic.loaded_scan_data
            if loaded_data is not None and loaded_data.timestamp_start is not None:
                ts = loaded_data.timestamp_start.strftime('%Y-%m-%d %H:%M')
                self._mw.setWindowTitle(f'Motor XY Scan - [Loaded: {ts}]')
            else:
                self._mw.setWindowTitle('Motor XY Scan - [Loaded Data]')
        else:
            # Reset window title
            self._mw.setWindowTitle('Motor XY Scan')

        # Clear selected pixel when switching data sources
        self._selected_pixel = None
        self._mw.pixel_spectrum_widget.clear()

        # Trigger display refresh
        self._update_display()
        self._update_progress()

    def _mode_changed(self, mode_text: str):
        """Handle scan mode change."""
        self._logic.set_scan_mode(mode_text)

        # Update display options based on mode
        self._mw.display_combo.clear()
        if mode_text == 'STEP_ODMR':
            self._mw.display_combo.addItems([
                'Center Frequency', 'Linewidth', 'Splitting', 'Fit Quality'
            ])
        elif mode_text == 'CONTINUOUS_FREQ_TRACK':
            self._mw.display_combo.addItems(['Absolute Frequency'])
        elif mode_text == 'POSITION_ONLY':
            self._mw.display_combo.addItems(['Scan Progress'])
        elif mode_text == 'KDC_HW_SYNC_MULTIRES':
            self._mw.display_combo.addItems(MULTIRES_DISPLAY_OPTIONS)
        else:
            self._mw.display_combo.addItems(['Mean Value'])

        # Show/hide lock status indicator based on mode
        show_lock_status = (mode_text == 'CONTINUOUS_FREQ_TRACK')
        self._mw.lock_status_label.setVisible(show_lock_status)
        if show_lock_status:
            self._mw.lock_status_label.setText('🔓 Unlocked')
            self._mw.lock_status_label.setStyleSheet('color: gray;')

        # Show/hide pixel spectrum panel based on mode (only for STEP_ODMR)
        is_step_odmr = (mode_text == 'STEP_ODMR')
        self._mw.pixel_spectrum_groupbox.setVisible(is_step_odmr)

        # Enable/disable Load button based on mode (only for STEP_ODMR when idle)
        from qudi.logic.motor_scan import ScanState
        is_idle = self._logic.scan_state == ScanState.IDLE
        self._mw.action_load.setEnabled(is_step_odmr and is_idle)

        # Clear spectrum when switching away from STEP_ODMR
        if not is_step_odmr:
            self._selected_pixel = None
            self._mw.pixel_spectrum_widget.clear()
    
    def _pattern_changed(self, pattern_text: str):
        """Handle scan pattern change."""
        self._logic.set_scan_pattern(pattern_text)
    
    def _display_channel_changed(self, channel: str):
        """Handle display channel change."""
        self._update_display()

    def _save_traces_toggled(self, checked: bool):
        """Handle the KDC_HW_SYNC 'save full traces' checkbox."""
        self._logic.set_save_full_traces(bool(checked))
    
    def _on_scan_state_changed(self, state):
        """Handle scan state change from logic."""
        from qudi.logic.motor_scan import ScanState

        # Check if we're in any active state (initializing, running, or paused)
        is_busy = state in (ScanState.INITIALIZING, ScanState.RUNNING, ScanState.PAUSED)
        is_running = state in (ScanState.RUNNING, ScanState.PAUSED)
        is_paused = state == ScanState.PAUSED
        is_initializing = state == ScanState.INITIALIZING

        # Clear pixel spectrum when a new scan starts
        if is_initializing:
            self._selected_pixel = None
            self._mw.pixel_spectrum_widget.clear()

        # Start button: checked when running/paused, but also when initializing
        self._mw.action_start_scan.setChecked(is_busy)
        # Disable start button during initialization (can't stop during init)
        self._mw.action_start_scan.setEnabled(not is_initializing)
        
        self._mw.action_pause_scan.setChecked(is_paused)
        self._mw.action_pause_scan.setEnabled(is_running)
        
        # Disable all settings during any busy state
        self._mw.x_start_spinbox.setEnabled(not is_busy)
        self._mw.x_stop_spinbox.setEnabled(not is_busy)
        self._mw.y_start_spinbox.setEnabled(not is_busy)
        self._mw.y_stop_spinbox.setEnabled(not is_busy)
        self._mw.x_points_spinbox.setEnabled(not is_busy)
        self._mw.y_points_spinbox.setEnabled(not is_busy)
        self._mw.mode_combo.setEnabled(not is_busy)
        self._mw.pattern_combo.setEnabled(not is_busy)
        self._mw.apply_settings_button.setEnabled(not is_busy)
        self._mw.action_home_stages.setEnabled(not is_busy)

        # Disable stage control during scan
        self._mw.move_x_spinbox.setEnabled(not is_busy)
        self._mw.move_y_spinbox.setEnabled(not is_busy)
        self._mw.move_button.setEnabled(not is_busy)
        self._mw.move_to_start_button.setEnabled(not is_busy)
        self._mw.stop_move_button.setEnabled(False)  # Stop button only for manual moves

        # Load button enabled only in STEP_ODMR mode when idle
        is_step_odmr = self._mw.mode_combo.currentText() == 'STEP_ODMR'
        self._mw.action_load.setEnabled(not is_busy and is_step_odmr)

        # Update status label with user-friendly text
        if is_initializing:
            self._mw.scan_status_label.setText('Initializing (homing/positioning)...')
        else:
            self._mw.scan_status_label.setText(state.name)
    
    def _on_scan_status_message(self, message: str):
        """Handle descriptive status text from logic (e.g. 'Scanning line 3/20...')."""
        self._mw.scan_status_label.setText(message)

    def _on_scan_data_updated(self):
        """Handle scan data update from logic."""
        self._update_display()
        self._update_progress()

        # Refresh pixel spectrum if a pixel is selected (updates with new fit data)
        if self._selected_pixel is not None:
            self._refresh_selected_pixel_spectrum()
    
    def _on_position_updated(self, position: Dict[str, float]):
        """Handle position update from logic."""
        x = position.get('x', 0) * 1000  # m to mm
        y = position.get('y', 0) * 1000
        self._mw.position_label.setText(f'Position: X={x:.3f}mm, Y={y:.3f}mm')
    
    def _on_settings_changed(self, settings: dict):
        """Handle settings change from logic."""
        self._restore_settings_from_logic()
    
    def _on_scan_completed(self, scan_data):
        """Handle scan completion."""
        self._mw.progress_bar.setValue(100)
        self._mw.scan_status_label.setText('Completed')
        self._update_display()
    
    def _on_save_state_changed(self, is_saving: bool):
        """Handle save state change from logic."""
        # Disable/enable save button during save operation
        self._mw.action_save.setEnabled(not is_saving)
        self._mw.save_nametag_lineedit.setEnabled(not is_saving)
        if is_saving:
            self._mw.scan_status_label.setText('Saving...')
        else:
            self._mw.scan_status_label.setText('Save complete')
    
    def _on_homing_state_changed(self, is_homing: bool):
        """Handle homing state change from logic."""
        # Disable all controls during homing (same as during scan initialization)
        self._mw.action_home_stages.setEnabled(not is_homing)
        self._mw.action_start_scan.setEnabled(not is_homing)
        self._mw.action_pause_scan.setEnabled(False)  # Can't pause during homing
        
        # Disable all settings during homing
        self._mw.x_start_spinbox.setEnabled(not is_homing)
        self._mw.x_stop_spinbox.setEnabled(not is_homing)
        self._mw.y_start_spinbox.setEnabled(not is_homing)
        self._mw.y_stop_spinbox.setEnabled(not is_homing)
        self._mw.x_points_spinbox.setEnabled(not is_homing)
        self._mw.y_points_spinbox.setEnabled(not is_homing)
        self._mw.mode_combo.setEnabled(not is_homing)
        self._mw.pattern_combo.setEnabled(not is_homing)
        self._mw.apply_settings_button.setEnabled(not is_homing)

        # Disable stage control during homing
        self._mw.move_x_spinbox.setEnabled(not is_homing)
        self._mw.move_y_spinbox.setEnabled(not is_homing)
        self._mw.move_button.setEnabled(not is_homing)
        self._mw.move_to_start_button.setEnabled(not is_homing)
        self._mw.stop_move_button.setEnabled(False)  # Can't stop homing with this button

        # Load button disabled during homing
        is_step_odmr = self._mw.mode_combo.currentText() == 'STEP_ODMR'
        self._mw.action_load.setEnabled(not is_homing and is_step_odmr)

        # Update status label
        if is_homing:
            self._mw.scan_status_label.setText('Homing stages...')
        else:
            self._mw.scan_status_label.setText('Homing complete')

    def _on_movement_state_changed(self, is_moving: bool):
        """Handle movement state change from logic."""
        # Disable all controls during movement (same as during homing)
        self._mw.action_home_stages.setEnabled(not is_moving)
        self._mw.action_start_scan.setEnabled(not is_moving)
        self._mw.action_pause_scan.setEnabled(False)  # Can't pause during movement

        # Disable all scan settings during movement
        self._mw.x_start_spinbox.setEnabled(not is_moving)
        self._mw.x_stop_spinbox.setEnabled(not is_moving)
        self._mw.y_start_spinbox.setEnabled(not is_moving)
        self._mw.y_stop_spinbox.setEnabled(not is_moving)
        self._mw.x_points_spinbox.setEnabled(not is_moving)
        self._mw.y_points_spinbox.setEnabled(not is_moving)
        self._mw.mode_combo.setEnabled(not is_moving)
        self._mw.pattern_combo.setEnabled(not is_moving)
        self._mw.apply_settings_button.setEnabled(not is_moving)

        # Disable stage control inputs during movement
        self._mw.move_x_spinbox.setEnabled(not is_moving)
        self._mw.move_y_spinbox.setEnabled(not is_moving)
        self._mw.move_button.setEnabled(not is_moving)
        self._mw.move_to_start_button.setEnabled(not is_moving)

        # Stop button is ONLY enabled during movement
        self._mw.stop_move_button.setEnabled(is_moving)

        # Load button disabled during movement
        is_step_odmr = self._mw.mode_combo.currentText() == 'STEP_ODMR'
        self._mw.action_load.setEnabled(not is_moving and is_step_odmr)

        # Update status label
        if is_moving:
            self._mw.scan_status_label.setText('Moving to position...')
        else:
            self._mw.scan_status_label.setText('Ready')

    def _move_to_position(self):
        """Move stages to the target position specified in spinboxes."""
        x = self._mw.move_x_spinbox.value() / 1000  # mm to m
        y = self._mw.move_y_spinbox.value() / 1000  # mm to m
        self._logic.move_to_position({'x': x, 'y': y})

    def _move_to_scan_start(self):
        """Move stages to the scan start position."""
        x = self._mw.x_start_spinbox.value() / 1000  # mm to m
        y = self._mw.y_start_spinbox.value() / 1000  # mm to m
        self._logic.move_to_position({'x': x, 'y': y})

    @QtCore.Slot()
    def _on_lock_lost(self):
        """Handle lock lost during CONTINUOUS_FREQ_TRACK scan."""
        # Show warning dialog
        QtWidgets.QMessageBox.warning(
            self._mw,
            'Lock Lost',
            'Frequency lock was lost during scan.\n\n'
            'The scan has been paused. Please:\n'
            '1. Check the ODMR Tracking GUI\n'
            '2. Re-enable the frequency lock\n'
            '3. Click Resume to continue scanning\n\n'
            'Note: Zero-crossing may update if you perform a new ODMR fit.',
            QtWidgets.QMessageBox.Ok
        )
        # Update lock status indicator
        self._mw.lock_status_label.setText('🔓 Lock LOST')
        self._mw.lock_status_label.setStyleSheet('color: red; font-weight: bold;')

    @QtCore.Slot(bool)
    def _on_lock_status_updated(self, locked: bool):
        """Handle lock status update from logic."""
        if locked:
            self._mw.lock_status_label.setText('🔒 Locked')
            self._mw.lock_status_label.setStyleSheet('color: green;')
        else:
            self._mw.lock_status_label.setText('🔓 Unlocked')
            self._mw.lock_status_label.setStyleSheet('color: orange;')

    def _multires_display_array(self, scan_data, label: str):
        """Build the (data, unit) to display for a KDC_HW_SYNC_MULTIRES channel.

        Individual channels map to a stored map (res{k}_corr in Hz, res{k}_err in
        LSB); the derived options compute the difference/sum of the two corrections.
        Returns (None, None) if the needed channel(s) are unavailable.
        """
        maps = scan_data.stream_data_mean
        if not maps:
            return None, None

        def _get(name):
            return maps.get(name)

        if label == 'Res0 Correction':
            return _get('res0_corr'), 'Hz'
        if label == 'Res1 Correction':
            return _get('res1_corr'), 'Hz'
        if label == 'Res0 Error':
            return _get('res0_err'), ''
        if label == 'Res1 Error':
            return _get('res1_err'), ''
        if label in ('Δ Correction (R1−R0)', 'Σ Correction (R0+R1)'):
            c0, c1 = _get('res0_corr'), _get('res1_corr')
            if c0 is None or c1 is None:
                return None, None
            if label.startswith('Δ'):
                return c1 - c0, 'Hz'
            return c0 + c1, 'Hz'
        # Fallback: first available channel
        first = next(iter(maps.values()), None)
        return (first, 'Hz') if first is not None else (None, None)

    def _update_display(self):
        """Update the scan image display."""
        scan_data = self._logic.display_scan_data
        if scan_data is None:
            return
        
        display_channel = self._mw.display_combo.currentText()
        
        # Get the appropriate data array based on display selection
        from qudi.logic.motor_scan import ScanMode
        
        if scan_data.scan_mode == ScanMode.POSITION_ONLY:
            # Display progress: 1.0 for visited points, 0.0 for unvisited
            if scan_data.is_2d:
                shape_2d = (scan_data.scan_resolution[0], scan_data.scan_resolution[1])
            else:
                shape_2d = (scan_data.scan_resolution[0],)

            data = np.zeros(shape_2d)
            # Mark visited points based on current_point_index
            for i in range(scan_data.current_point_index):
                grid_idx = scan_data.point_index_to_grid_index(i)
                if scan_data.is_2d:
                    data[grid_idx[0], grid_idx[1]] = 1.0
                else:
                    data[grid_idx[0]] = 1.0

            unit = ''
            display_channel = 'Scan Progress'
        elif scan_data.scan_mode == ScanMode.STEP_ODMR:
            if display_channel == 'Center Frequency' and scan_data.center_frequency is not None:
                data = scan_data.center_frequency
                unit = 'Hz'
            elif display_channel == 'Linewidth' and scan_data.linewidth is not None:
                data = scan_data.linewidth
                unit = 'Hz'
            elif display_channel == 'Splitting' and scan_data.splitting is not None:
                data = scan_data.splitting
                unit = 'Hz'
            elif display_channel == 'Fit Quality' and scan_data.fit_quality is not None:
                data = scan_data.fit_quality
                unit = ''
            else:
                return
        elif scan_data.scan_mode == ScanMode.CONTINUOUS_FREQ_TRACK:
            # Frequency tracking mode - display in GHz for readability
            if scan_data.stream_data_mean and 'absolute_frequency' in scan_data.stream_data_mean:
                data = scan_data.stream_data_mean['absolute_frequency'] / 1e9  # Hz to GHz
                unit = 'GHz'
            else:
                return
        elif scan_data.scan_mode == ScanMode.KDC_HW_SYNC_MULTIRES:
            data, unit = self._multires_display_array(scan_data, display_channel)
            if data is None:
                return
        else:
            # Continuous stream mode
            if scan_data.stream_data_mean:
                channel_name = list(scan_data.stream_data_mean.keys())[0]
                data = scan_data.stream_data_mean[channel_name]
                unit = 'V'
            else:
                return
        
        # Flip data array for display when scan direction is reversed.
        #
        # The image widget's set_image_extent() normalizes ranges to (min, max),
        # which means pyqtgraph always maps array[0,0] to (x_min, y_min).
        # But our grid index convention is: ix=0 corresponds to x_start, iy=0 to y_start.
        # When start > stop (reversed scan), we must flip the data so that:
        #   - Data at grid iy=0 (y_start) appears at y_start position, not y_min
        #   - Data at grid ix=0 (x_start) appears at x_start position, not x_min
        #
        # This flip is ONLY for visualization. Data storage and click-to-data
        # lookup use the original (non-flipped) arrays with the original scan_range.

        data_display = data
        if scan_data.is_2d:
            x_range = scan_data.scan_range[0]
            y_range = scan_data.scan_range[1]

            x_reversed = x_range[0] > x_range[1]  # x_start > x_stop
            y_reversed = y_range[0] > y_range[1]  # y_start > y_stop

            if x_reversed or y_reversed:
                data_display = data.copy()
                if x_reversed:
                    data_display = data_display[::-1, :]  # Flip along X (first axis)
                if y_reversed:
                    data_display = data_display[:, ::-1]  # Flip along Y (second axis)

        # Update image
        self._mw.image_widget.set_image(data_display)
        self._mw.image_widget.set_data_label(label=display_channel, unit=unit)

        # Set proper extent (widget normalizes to min/max internally)
        if scan_data.is_2d:
            self._mw.image_widget.set_image_extent((x_range, y_range), adjust_for_px_size=True)
    
    def _update_progress(self):
        """Update progress bar."""
        scan_data = self._logic.display_scan_data
        if scan_data is not None:
            progress = int(scan_data.progress * 100)
            self._mw.progress_bar.setValue(progress)

    def _on_scene_mouse_clicked(self, event) -> None:
        """
        Handle mouse click on scan image scene.

        Uses pyqtgraph scene's sigMouseClicked for reliable click detection
        that works regardless of zoom level or view transformations.

        Args:
            event: MouseClickEvent from pyqtgraph scene
        """
        # Only handle left clicks
        if event.button() != QtCore.Qt.LeftButton:
            return

        # Get the ViewBox to convert coordinates
        view_box = self._mw.image_widget.plot_widget.getViewBox()

        # Check if click is within the ViewBox
        scene_pos = event.scenePos()
        if not view_box.sceneBoundingRect().contains(scene_pos):
            return

        # Map scene coordinates to data coordinates
        data_pos = view_box.mapSceneToView(scene_pos)
        x_pos = data_pos.x()
        y_pos = data_pos.y()

        # Handle the pixel click
        self._handle_pixel_click(x_pos, y_pos)

    def _handle_pixel_click(self, x_pos: float, y_pos: float) -> None:
        """
        Handle a pixel click at the given data coordinates.

        Args:
            x_pos: X position in meters
            y_pos: Y position in meters
        """
        from qudi.logic.motor_scan import ScanMode

        # Only handle in STEP_ODMR mode
        scan_data = self._logic.display_scan_data
        if scan_data is None or scan_data.scan_mode != ScanMode.STEP_ODMR:
            return

        # Convert position to grid index
        grid_idx = self._position_to_grid_index(x_pos, y_pos)
        if grid_idx is None:
            return

        ix, iy = grid_idx
        self._selected_pixel = grid_idx

        # Get ODMR data for this pixel
        linear_idx = self._grid_index_to_linear_index(ix, iy)

        # Check if data exists for this pixel
        if scan_data.odmr_raw_per_pixel is None:
            self._mw.pixel_spectrum_widget.clear()
            return

        if linear_idx >= len(scan_data.odmr_raw_per_pixel):
            self._mw.pixel_spectrum_widget.clear()
            return

        raw_data = scan_data.odmr_raw_per_pixel[linear_idx]
        if raw_data is None:
            self._mw.pixel_spectrum_widget.clear()
            return

        # Get fit result (may be None if not fitted yet)
        fit_result = None
        if scan_data.odmr_fit_results is not None:
            if ix < len(scan_data.odmr_fit_results):
                if iy < len(scan_data.odmr_fit_results[ix]):
                    fit_result = scan_data.odmr_fit_results[ix][iy]

        # Prepare pixel info
        pixel_info = {
            'grid_index': (ix, iy),
            'position_mm': (x_pos * 1000, y_pos * 1000),  # m to mm
        }

        # Update spectrum widget
        self._mw.pixel_spectrum_widget.set_pixel_data(
            frequency_data=raw_data.get('frequency_data'),
            signal_data=raw_data.get('signal_data'),
            fit_result=fit_result,
            pixel_info=pixel_info
        )

    def _position_to_grid_index(self, x_pos: float, y_pos: float) -> Optional[Tuple[int, int]]:
        """
        Convert data coordinates (meters) to grid indices (ix, iy).

        Args:
            x_pos: X position in meters
            y_pos: Y position in meters

        Returns:
            Tuple (ix, iy) or None if position is far out of bounds
        """
        scan_data = self._logic.display_scan_data
        if scan_data is None:
            return None

        x_range = scan_data.scan_range[0]  # (start, stop) in meters
        y_range = scan_data.scan_range[1]
        nx, ny = scan_data.scan_resolution

        # Calculate pixel size for tolerance
        x_span = abs(x_range[1] - x_range[0])
        y_span = abs(y_range[1] - y_range[0])
        pixel_size_x = x_span / max(1, nx - 1) if nx > 1 else x_span
        pixel_size_y = y_span / max(1, ny - 1) if ny > 1 else y_span

        # Add tolerance of 1 pixel to bounds check (allows clicking on edge pixels)
        x_min, x_max = min(x_range), max(x_range)
        y_min, y_max = min(y_range), max(y_range)
        tolerance_x = pixel_size_x * 0.6
        tolerance_y = pixel_size_y * 0.6

        # Check bounds with tolerance
        if not (x_min - tolerance_x <= x_pos <= x_max + tolerance_x and
                y_min - tolerance_y <= y_pos <= y_max + tolerance_y):
            return None

        # Linear interpolation to nearest index
        if x_range[1] != x_range[0]:
            ix = int(round((x_pos - x_range[0]) / (x_range[1] - x_range[0]) * (nx - 1)))
        else:
            ix = 0
        if y_range[1] != y_range[0]:
            iy = int(round((y_pos - y_range[0]) / (y_range[1] - y_range[0]) * (ny - 1)))
        else:
            iy = 0

        # Clamp to valid range
        ix = max(0, min(nx - 1, ix))
        iy = max(0, min(ny - 1, iy))

        return (ix, iy)

    def _grid_index_to_linear_index(self, ix: int, iy: int) -> int:
        """
        Convert grid indices to linear index accounting for scan pattern.

        Args:
            ix: X grid index
            iy: Y grid index

        Returns:
            Linear index into odmr_raw_per_pixel list
        """
        from qudi.logic.motor_scan import ScanPattern

        scan_data = self._logic.display_scan_data
        nx = scan_data.scan_resolution[0]
        pattern = scan_data.scan_pattern

        if pattern in (ScanPattern.LINE_BY_LINE_X, ScanPattern.SNAKE_X):
            # Fast axis is X
            if pattern == ScanPattern.SNAKE_X and iy % 2 == 1:
                ix_in_line = nx - 1 - ix  # Reverse for odd lines
            else:
                ix_in_line = ix
            return iy * nx + ix_in_line
        else:
            # Fast axis is Y (LINE_BY_LINE_Y or SNAKE_Y)
            ny = scan_data.scan_resolution[1]
            if pattern == ScanPattern.SNAKE_Y and ix % 2 == 1:
                iy_in_line = ny - 1 - iy
            else:
                iy_in_line = iy
            return ix * ny + iy_in_line

    def _refresh_selected_pixel_spectrum(self) -> None:
        """Refresh the spectrum display for the currently selected pixel."""
        if self._selected_pixel is None:
            return

        from qudi.logic.motor_scan import ScanMode

        scan_data = self._logic.display_scan_data
        if scan_data is None or scan_data.scan_mode != ScanMode.STEP_ODMR:
            return

        ix, iy = self._selected_pixel
        linear_idx = self._grid_index_to_linear_index(ix, iy)

        # Check if data exists for this pixel
        if scan_data.odmr_raw_per_pixel is None:
            return

        if linear_idx >= len(scan_data.odmr_raw_per_pixel):
            return

        raw_data = scan_data.odmr_raw_per_pixel[linear_idx]
        if raw_data is None:
            return

        # Get fit result
        fit_result = None
        if scan_data.odmr_fit_results is not None:
            if ix < len(scan_data.odmr_fit_results):
                if iy < len(scan_data.odmr_fit_results[ix]):
                    fit_result = scan_data.odmr_fit_results[ix][iy]

        # Get position from scan data
        x_range = scan_data.scan_range[0]
        y_range = scan_data.scan_range[1]
        nx, ny = scan_data.scan_resolution
        if nx > 1:
            x_pos = x_range[0] + ix * (x_range[1] - x_range[0]) / (nx - 1)
        else:
            x_pos = x_range[0]
        if ny > 1:
            y_pos = y_range[0] + iy * (y_range[1] - y_range[0]) / (ny - 1)
        else:
            y_pos = y_range[0]

        pixel_info = {
            'grid_index': (ix, iy),
            'position_mm': (x_pos * 1000, y_pos * 1000),
        }

        self._mw.pixel_spectrum_widget.set_pixel_data(
            frequency_data=raw_data.get('frequency_data'),
            signal_data=raw_data.get('signal_data'),
            fit_result=fit_result,
            pixel_info=pixel_info
        )
