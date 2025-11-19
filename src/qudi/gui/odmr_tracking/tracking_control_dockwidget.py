# -*- coding: utf-8 -*-

"""
This file contains the tracking control dock widget for ODMR tracking GUI.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-core/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

from PySide2 import QtCore, QtWidgets, QtGui


class TrackingControlDockWidget(QtWidgets.QDockWidget):
    """
    Dock widget for controlling ODMR frequency tracking parameters.

    Provides UI controls for:
    - Enabling/disabling tracking
    - Selecting tracking target (peak/dip)
    - Setting tracking threshold
    - Configuring tracking window size
    - Displaying current tracking status
    """

    # Signals
    sigToggleTracking = QtCore.Signal(bool)
    sigTrackingParametersChanged = QtCore.Signal(dict)

    def __init__(self, parent=None):
        super().__init__('Tracking Controls', parent)

        # Create main widget and layout
        self._widget = QtWidgets.QWidget()
        self._layout = QtWidgets.QVBoxLayout()
        self._widget.setLayout(self._layout)
        self.setWidget(self._widget)

        # Setup UI components
        self._setup_ui()

        # Set widget properties
        self.setFeatures(
            QtWidgets.QDockWidget.DockWidgetClosable |
            QtWidgets.QDockWidget.DockWidgetMovable |
            QtWidgets.QDockWidget.DockWidgetFloatable
        )
        self.setAllowedAreas(
            QtCore.Qt.LeftDockWidgetArea |
            QtCore.Qt.RightDockWidgetArea
        )

    def _setup_ui(self):
        """
        Create all UI elements for the tracking control panel.
        """
        # === Tracking Enable/Disable ===
        enable_group = QtWidgets.QGroupBox('Tracking Control')
        enable_layout = QtWidgets.QVBoxLayout()

        self.tracking_button = QtWidgets.QPushButton('Start Tracking')
        self.tracking_button.setCheckable(True)
        self.tracking_button.setStyleSheet("""
            QPushButton:checked {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
            }
        """)
        self.tracking_button.clicked.connect(self._on_tracking_button_clicked)
        enable_layout.addWidget(self.tracking_button)

        enable_group.setLayout(enable_layout)
        self._layout.addWidget(enable_group)

        # === Tracking Parameters ===
        params_group = QtWidgets.QGroupBox('Tracking Parameters')
        params_layout = QtWidgets.QFormLayout()

        # Target selection (peak or dip)
        self.target_combo = QtWidgets.QComboBox()
        self.target_combo.addItems(['Dip (Minimum)', 'Peak (Maximum)'])
        self.target_combo.currentIndexChanged.connect(self._on_parameters_changed)
        params_layout.addRow('Track Target:', self.target_combo)

        # Threshold setting
        self.threshold_spinbox = QtWidgets.QDoubleSpinBox()
        self.threshold_spinbox.setRange(0.0, 1.0)
        self.threshold_spinbox.setSingleStep(0.001)
        self.threshold_spinbox.setDecimals(4)
        self.threshold_spinbox.setValue(0.01)
        self.threshold_spinbox.setSuffix('')
        self.threshold_spinbox.setToolTip('Minimum contrast/change threshold for valid tracking')
        self.threshold_spinbox.valueChanged.connect(self._on_parameters_changed)
        params_layout.addRow('Threshold:', self.threshold_spinbox)

        # Window size setting
        self.window_spinbox = QtWidgets.QSpinBox()
        self.window_spinbox.setRange(3, 51)
        self.window_spinbox.setSingleStep(2)
        self.window_spinbox.setValue(5)
        self.window_spinbox.setSuffix(' pts')
        self.window_spinbox.setToolTip('Number of points around feature for analysis')
        self.window_spinbox.valueChanged.connect(self._on_parameters_changed)
        params_layout.addRow('Window Size:', self.window_spinbox)

        params_group.setLayout(params_layout)
        self._layout.addWidget(params_group)

        # === Tracking Status/Info ===
        status_group = QtWidgets.QGroupBox('Tracking Status')
        status_layout = QtWidgets.QFormLayout()

        # Current tracked frequency
        self.freq_label = QtWidgets.QLabel('---')
        self.freq_label.setStyleSheet('QLabel { font-family: monospace; }')
        status_layout.addRow('Frequency:', self.freq_label)

        # Contrast value
        self.contrast_label = QtWidgets.QLabel('---')
        self.contrast_label.setStyleSheet('QLabel { font-family: monospace; }')
        status_layout.addRow('Contrast:', self.contrast_label)

        # Scan count at tracking point
        self.scan_label = QtWidgets.QLabel('---')
        self.scan_label.setStyleSheet('QLabel { font-family: monospace; }')
        status_layout.addRow('Scan #:', self.scan_label)

        # History length
        self.history_label = QtWidgets.QLabel('0')
        self.history_label.setStyleSheet('QLabel { font-family: monospace; }')
        status_layout.addRow('History:', self.history_label)

        status_group.setLayout(status_layout)
        self._layout.addWidget(status_group)

        # Add stretch to push everything to top
        self._layout.addStretch()

    # ===========================================================================
    # User Interaction Handlers
    # ===========================================================================

    @QtCore.Slot(bool)
    def _on_tracking_button_clicked(self, checked):
        """
        Handle tracking enable/disable button click.
        """
        if checked:
            self.tracking_button.setText('Stop Tracking')
        else:
            self.tracking_button.setText('Start Tracking')

        self.sigToggleTracking.emit(checked)

    @QtCore.Slot()
    def _on_parameters_changed(self):
        """
        Handle any tracking parameter change and emit update signal.
        """
        params = {
            'target': 'dip' if self.target_combo.currentIndex() == 0 else 'peak',
            'threshold': self.threshold_spinbox.value(),
            'window_size': self.window_spinbox.value()
        }
        self.sigTrackingParametersChanged.emit(params)

    # ===========================================================================
    # Public Methods (called by main GUI)
    # ===========================================================================

    def set_tracking_state(self, tracking_active):
        """
        Update the tracking button state.

        @param bool tracking_active: Whether tracking is currently active
        """
        self.tracking_button.blockSignals(True)
        self.tracking_button.setChecked(tracking_active)
        if tracking_active:
            self.tracking_button.setText('Stop Tracking')
        else:
            self.tracking_button.setText('Start Tracking')
        self.tracking_button.blockSignals(False)

    def update_tracking_info(self, tracking_data):
        """
        Update the status display with current tracking information.

        @param dict tracking_data: Dictionary with tracking data
                                   {'frequency', 'contrast', 'scan_count', etc.}
        """
        # Update frequency
        freq = tracking_data.get('frequency')
        if freq is not None:
            self.freq_label.setText(f'{freq/1e9:.6f} GHz')

        # Update contrast
        contrast = tracking_data.get('contrast')
        if contrast is not None:
            self.contrast_label.setText(f'{contrast:.4f}')

        # Update scan count
        scan_count = tracking_data.get('scan_count')
        if scan_count is not None:
            self.scan_label.setText(f'{scan_count}')

    def update_parameters(self, params):
        """
        Update the parameter widgets to match logic state.

        @param dict params: Dictionary of tracking parameters
        """
        # Block signals while updating to avoid feedback loop
        self.target_combo.blockSignals(True)
        self.threshold_spinbox.blockSignals(True)
        self.window_spinbox.blockSignals(True)

        # Update target
        target = params.get('target')
        if target == 'dip':
            self.target_combo.setCurrentIndex(0)
        elif target == 'peak':
            self.target_combo.setCurrentIndex(1)

        # Update threshold
        threshold = params.get('threshold')
        if threshold is not None:
            self.threshold_spinbox.setValue(threshold)

        # Update window size
        window_size = params.get('window_size')
        if window_size is not None:
            self.window_spinbox.setValue(window_size)

        # Re-enable signals
        self.target_combo.blockSignals(False)
        self.threshold_spinbox.blockSignals(False)
        self.window_spinbox.blockSignals(False)

    def set_history_length(self, length):
        """
        Update the displayed tracking history length.

        @param int length: Number of points in tracking history
        """
        self.history_label.setText(f'{length}')
