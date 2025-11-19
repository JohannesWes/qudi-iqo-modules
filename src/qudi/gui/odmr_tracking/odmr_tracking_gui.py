# -*- coding: utf-8 -*-
"""
ODMR Frequency Tracking GUI Module

This GUI module extends the standard ODMR GUI with frequency tracking capabilities:
- Full ODMR scan control and visualization (inherited from OdmrGui)
- Linear fit controls for error signal extraction
- Frequency lock controls (Integral and PI modes)
- Real-time error signal and frequency monitoring

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

__all__ = ['OdmrTrackingGui']

import pyqtgraph as pg
import numpy as np
from PySide2 import QtCore, QtWidgets, QtGui

from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.util.colordefs import QudiPalettePale as palette
from qudi.util.widgets.scientific_spinbox import ScienDSpinBox
from qudi.gui.odmr.odmrgui import OdmrGui


class OdmrTrackingGui(OdmrGui):
    """
    Extended ODMR GUI with frequency tracking controls.

    Inherits all ODMR scan functionality and adds:
    - Resonance fitting controls
    - Frequency lock controls (Integral and PI modes)
    - Real-time error signal monitoring
    - Real-time frequency drift monitoring (future)

    Example config:

        odmr_tracking_gui:
            module.Class: 'odmr_tracking.odmr_tracking_gui.OdmrTrackingGui'
            connect:
                odmr_logic: 'odmr_frequency_tracking_logic'
    """

    # =========================================================================
    # Status Variables (tracking-specific)
    # =========================================================================

    _fit_freq_min = StatusVar('fit_freq_min', default=2.86e9)
    _fit_freq_max = StatusVar('fit_freq_max', default=2.88e9)
    _lock_bandwidth = StatusVar('lock_bandwidth', default=300.0)
    _lock_zero_ratio = StatusVar('lock_zero_ratio', default=3.0)
    _lock_mode = StatusVar('lock_mode', default='integral')  # 'integral' or 'pi'
    _stream_mode = StatusVar('stream_mode', default='error')  # 'error' or 'correction'

    # =========================================================================
    # Signals (tracking-specific)
    # =========================================================================

    sigDoFit = QtCore.Signal(float, float)  # freq_min, freq_max
    sigConfigureLock = QtCore.Signal(str, float, float)  # mode, bandwidth, zero_ratio
    sigClearIntegrator = QtCore.Signal()
    sigSetStreamEnabled = QtCore.Signal(bool)  # stream control (independent of lock)
    sigSetStreamMode = QtCore.Signal(str)  # 'error' or 'correction'
    sigSetLockEnabled = QtCore.Signal(bool)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Tracking-specific widgets (add to parent's widgets)
        self._fit_region = None
        self._tracking_fit_curve = None
        self._error_plot_widget = None
        self._freq_plot_widget = None
        self._error_curve = None
        self._freq_curve = None
        self._fit_controls_dock = None
        self._lock_controls_dock = None
        self._time_series_dock = None
        self._status_labels = {}

    def on_activate(self):
        """Extended activation - calls parent then adds tracking GUI"""
        # Call parent activation (creates all standard ODMR GUI elements)
        super().on_activate()

        # Hide ODMR scanning multiline plot and colorbar to compact GUI
        self._hide_unused_parent_elements()

        # Add tracking-specific GUI components
        self._setup_tracking_gui()
        self._connect_tracking_signals()
        self._restore_tracking_settings()

        # Update window title
        self._mw.setWindowTitle('ODMR Frequency Tracking')
        
        # Restore default view (includes tracking docks)
        self.restore_default_view()

        self.log.info('ODMR Tracking GUI activated')

    def on_deactivate(self):
        """Extended deactivation - cleanup tracking GUI then call parent"""
        # Disconnect tracking signals
        self._disconnect_tracking_signals()

        # Call parent deactivation
        super().on_deactivate()

        self.log.info('ODMR Tracking GUI deactivated')

    def _hide_unused_parent_elements(self):
        """Hide GUI elements from parent OdmrGui that are not used in tracking mode"""
        # Hide ODMR fitting dock widget - we use our own linear fit controls
        if hasattr(self, '_fit_dockwidget') and self._fit_dockwidget is not None:
            self._fit_dockwidget.hide()
            self._mw.removeDockWidget(self._fit_dockwidget)

        # Hide menu actions related to ODMR fitting
        if hasattr(self._mw, 'action_show_fit_configuration'):
            self._mw.action_show_fit_configuration.setVisible(False)

        # Hide the matrix plot (scanning multiline) and colorbar from plot widget
        plot_widget = self._plot_widget
        if hasattr(plot_widget, '_image_widget'):
            plot_widget._image_widget.hide()
        if hasattr(plot_widget, '_colorbar'):
            plot_widget._colorbar.hide()

    # =========================================================================
    # Tracking GUI Setup
    # =========================================================================

    def _setup_tracking_gui(self):
        """Create and integrate tracking-specific GUI components"""
        # Add fit region overlay to existing ODMR plot
        self._add_fit_region_to_plot()

        # Create tracking control dock widgets
        self._create_fit_controls_dock()
        self._create_lock_controls_dock()
        self._create_time_series_dock()

    def _add_fit_region_to_plot(self):
        """Add fit region indicator to the inherited ODMR plot"""
        # Access the plot widget from parent class
        # The parent OdmrGui has self._plot_widget which is an OdmrPlotWidget
        # Inside OdmrPlotWidget, the actual pg.PlotWidget is _plot_widget
        plot_widget = self._plot_widget._plot_widget  # Get the pyqtgraph PlotWidget
        plot_item = plot_widget.getPlotItem()

        # Create fit region indicator
        self._fit_region = pg.LinearRegionItem(
            brush=pg.mkBrush(200, 200, 200, 50),
            movable=True
        )
        self._fit_region.sigRegionChanged.connect(self._on_fit_region_changed)
        plot_item.addItem(self._fit_region, ignoreBounds=True)

        # Create fit curve overlay
        self._tracking_fit_curve = pg.PlotDataItem(
            pen=pg.mkPen(palette.c2, width=3, style=QtCore.Qt.DashLine)
        )
        plot_item.addItem(self._tracking_fit_curve)

    def _create_fit_controls_dock(self):
        """Create fit control dock widget"""
        from qudi.util.widgets.advanced_dockwidget import AdvancedDockWidget

        self._fit_controls_dock = AdvancedDockWidget('Fit Controls', parent=self._mw)
        self._fit_controls_dock.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable | QtWidgets.QDockWidget.DockWidgetFloatable
        )

        # Create widget contents
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QFormLayout()
        widget.setLayout(layout)

        # Frequency range inputs
        self._freq_min_spinbox = ScienDSpinBox()
        self._freq_min_spinbox.setRange(0, 1e12)
        self._freq_min_spinbox.setSuffix('Hz')
        self._freq_min_spinbox.setDecimals(6)
        self._freq_min_spinbox.setMinimumWidth(120)

        self._freq_max_spinbox = ScienDSpinBox()
        self._freq_max_spinbox.setRange(0, 1e12)
        self._freq_max_spinbox.setSuffix('Hz')
        self._freq_max_spinbox.setDecimals(6)
        self._freq_max_spinbox.setMinimumWidth(120)

        # Connect spinbox signals to update fit region visualization
        self._freq_min_spinbox.valueChanged.connect(self._update_fit_region_from_spinboxes)
        self._freq_max_spinbox.valueChanged.connect(self._update_fit_region_from_spinboxes)

        # Fit button
        self._fit_button = QtWidgets.QPushButton('Fit Resonance')
        self._fit_button.clicked.connect(self._do_fit)

        # Fit results display
        self._fit_slope_label = QtWidgets.QLabel('--')
        self._fit_r2_label = QtWidgets.QLabel('--')

        # Layout
        layout.addRow('Freq Min:', self._freq_min_spinbox)
        layout.addRow('Freq Max:', self._freq_max_spinbox)
        layout.addRow(self._fit_button)
        layout.addRow('Slope [LSB/Hz]:', self._fit_slope_label)
        layout.addRow('R²:', self._fit_r2_label)

        self._fit_controls_dock.setWidget(widget)
        self._mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, self._fit_controls_dock)

    def _create_lock_controls_dock(self):
        """Create frequency lock control dock widget"""
        from qudi.util.widgets.advanced_dockwidget import AdvancedDockWidget

        self._lock_controls_dock = AdvancedDockWidget('Lock Controls', parent=self._mw)
        self._lock_controls_dock.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable | QtWidgets.QDockWidget.DockWidgetFloatable
        )

        # Create widget contents
        widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QVBoxLayout()
        widget.setLayout(main_layout)

        # Mode selection
        mode_group = QtWidgets.QGroupBox('Lock Mode')
        mode_layout = QtWidgets.QVBoxLayout()
        self._integral_radio = QtWidgets.QRadioButton('Integral')
        self._pi_radio = QtWidgets.QRadioButton('PI')
        self._integral_radio.setChecked(True)
        self._integral_radio.toggled.connect(self._on_lock_mode_changed)
        mode_layout.addWidget(self._integral_radio)
        mode_layout.addWidget(self._pi_radio)
        mode_group.setLayout(mode_layout)
        main_layout.addWidget(mode_group)

        # Lock parameters
        params_layout = QtWidgets.QFormLayout()

        self._bandwidth_spinbox = ScienDSpinBox()
        self._bandwidth_spinbox.setRange(0.1, 1e6)
        self._bandwidth_spinbox.setSuffix('Hz')
        self._bandwidth_spinbox.setValue(300.0)
        self._bandwidth_spinbox.setDecimals(1)
        self._bandwidth_spinbox.setMinimumWidth(120)

        self._zero_ratio_spinbox = QtWidgets.QDoubleSpinBox()
        self._zero_ratio_spinbox.setRange(2.0, 4.0)
        self._zero_ratio_spinbox.setSingleStep(0.1)
        self._zero_ratio_spinbox.setValue(3.0)
        self._zero_ratio_spinbox.setDecimals(1)
        self._zero_ratio_spinbox.setEnabled(False)  # Only for PI mode
        self._zero_ratio_spinbox.setToolTip(
            'PI zero placement ratio α\n'
            'Zero freq = Bandwidth / α\n'
            '  2.0: Aggressive (faster, may overshoot)\n'
            '  3.0: Balanced (recommended)\n'
            '  4.0: Conservative (slower, stable)'
        )

        params_layout.addRow('Bandwidth:', self._bandwidth_spinbox)
        params_layout.addRow('Zero Ratio (α):', self._zero_ratio_spinbox)

        # Control buttons
        button_layout = QtWidgets.QHBoxLayout()

        self._configure_button = QtWidgets.QPushButton('Configure')
        self._configure_button.clicked.connect(self._configure_lock)

        self._clear_button = QtWidgets.QPushButton('Clear')
        self._clear_button.clicked.connect(self._clear_integrator)
        self._clear_button.setEnabled(False)  # Only when lock is configured

        button_layout.addWidget(self._configure_button)
        button_layout.addWidget(self._clear_button)

        # Stream control buttons (independent of lock)
        stream_layout = QtWidgets.QHBoxLayout()

        self._start_stream_button = QtWidgets.QPushButton('Start Stream')
        self._start_stream_button.clicked.connect(lambda: self._set_stream_enabled(True))
        self._start_stream_button.setEnabled(True)

        self._stop_stream_button = QtWidgets.QPushButton('Stop Stream')
        self._stop_stream_button.clicked.connect(lambda: self._set_stream_enabled(False))
        self._stop_stream_button.setEnabled(False)

        stream_layout.addWidget(self._start_stream_button)
        stream_layout.addWidget(self._stop_stream_button)

        # Enable/Disable lock buttons
        enable_layout = QtWidgets.QHBoxLayout()

        self._enable_button = QtWidgets.QPushButton('Enable Lock')
        self._enable_button.clicked.connect(lambda: self._set_lock_enabled(True))
        self._enable_button.setEnabled(False)  # Only when configured

        self._disable_button = QtWidgets.QPushButton('Disable Lock')
        self._disable_button.clicked.connect(lambda: self._set_lock_enabled(False))
        self._disable_button.setEnabled(False)

        enable_layout.addWidget(self._enable_button)
        enable_layout.addWidget(self._disable_button)

        # Status indicators
        status_layout = QtWidgets.QFormLayout()

        self._status_labels['locked'] = QtWidgets.QLabel('OFF')
        self._status_labels['saturated'] = QtWidgets.QLabel('--')
        self._status_labels['error'] = QtWidgets.QLabel('--')
        self._status_labels['correction'] = QtWidgets.QLabel('--')

        status_layout.addRow('Locked:', self._status_labels['locked'])
        status_layout.addRow('Saturated:', self._status_labels['saturated'])
        status_layout.addRow('Error [LSB]:', self._status_labels['error'])
        status_layout.addRow('Correction [Hz]:', self._status_labels['correction'])

        # Assemble main layout
        main_layout.addLayout(params_layout)
        main_layout.addLayout(button_layout)
        main_layout.addWidget(QtWidgets.QLabel('Error Signal Streaming:'))
        main_layout.addLayout(stream_layout)
        main_layout.addWidget(QtWidgets.QLabel('Frequency Lock:'))
        main_layout.addLayout(enable_layout)
        main_layout.addWidget(QtWidgets.QLabel('Status:'))
        main_layout.addLayout(status_layout)
        main_layout.addStretch()

        self._lock_controls_dock.setWidget(widget)
        self._mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, self._lock_controls_dock)

    def _create_time_series_dock(self):
        """Create time series plots dock widget"""
        from qudi.util.widgets.advanced_dockwidget import AdvancedDockWidget

        self._time_series_dock = AdvancedDockWidget('Time Series', parent=self._mw)
        self._time_series_dock.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable | QtWidgets.QDockWidget.DockWidgetFloatable
        )

        # Create widget contents
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        widget.setLayout(layout)

        # Add stream mode selection controls
        mode_group = QtWidgets.QGroupBox('Stream Mode')
        mode_layout = QtWidgets.QHBoxLayout()

        self._error_mode_radio = QtWidgets.QRadioButton('Error Signal (LSB)')
        self._correction_mode_radio = QtWidgets.QRadioButton('Frequency Correction (Hz)')
        self._error_mode_radio.setChecked(True)
        self._error_mode_radio.toggled.connect(self._on_stream_mode_changed)

        mode_layout.addWidget(self._error_mode_radio)
        mode_layout.addWidget(self._correction_mode_radio)
        mode_group.setLayout(mode_layout)

        layout.addWidget(mode_group)

        # Signal plot (dynamically labeled based on mode)
        self._error_plot_widget = pg.PlotWidget()
        self._error_plot_widget.setLabel('bottom', 'Time', units='s')
        self._error_plot_widget.setLabel('left', 'Signal')  # Dynamic label
        self._error_plot_widget.showGrid(x=True, y=True)
        self._error_plot_widget.setMinimumHeight(150)

        self._error_curve = pg.PlotCurveItem(
            pen=pg.mkPen(palette.c3, width=2),
            clipToView=True,
            downsampleMethod='subsample',
            autoDownsample=True
        )
        self._error_plot_widget.addItem(self._error_curve)

        # Add plot to layout
        layout.addWidget(self._error_plot_widget)

        self._time_series_dock.setWidget(widget)
        self._mw.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self._time_series_dock)

    # =========================================================================
    # Signal Connections
    # =========================================================================

    def _connect_tracking_signals(self):
        """Connect signals from tracking logic to GUI"""
        logic = self._odmr_logic()

        # Connect tracking-specific signals
        logic.sigFitCompleted.connect(
            self._update_fit_display,
            QtCore.Qt.QueuedConnection
        )
        logic.sigFitFailed.connect(
            self._on_fit_failed,
            QtCore.Qt.QueuedConnection
        )
        logic.sigStreamStateChanged.connect(
            self._update_stream_state,
            QtCore.Qt.QueuedConnection
        )
        logic.sigLockStateChanged.connect(
            self._update_lock_state,
            QtCore.Qt.QueuedConnection
        )
        logic.sigLockStatusUpdated.connect(
            self._update_lock_status,
            QtCore.Qt.QueuedConnection
        )
        logic.sigErrorDataUpdated.connect(
            self._update_error_plot,
            QtCore.Qt.QueuedConnection
        )
        logic.sigStreamModeChanged.connect(
            self._update_stream_mode_ui,
            QtCore.Qt.QueuedConnection
        )

        # Connect GUI signals to logic
        self.sigDoFit.connect(
            lambda freq_min, freq_max: logic.fit_resonance(freq_min, freq_max),
            QtCore.Qt.QueuedConnection
        )
        self.sigConfigureLock.connect(
            self._handle_configure_lock,
            QtCore.Qt.QueuedConnection
        )
        self.sigClearIntegrator.connect(
            logic.clear_integrator,
            QtCore.Qt.QueuedConnection
        )
        self.sigSetStreamEnabled.connect(
            self._handle_set_stream_enabled,
            QtCore.Qt.QueuedConnection
        )
        self.sigSetStreamMode.connect(
            self._handle_set_stream_mode,
            QtCore.Qt.QueuedConnection
        )
        self.sigSetLockEnabled.connect(
            self._handle_set_lock_enabled,
            QtCore.Qt.QueuedConnection
        )

    def _disconnect_tracking_signals(self):
        """Disconnect signals from tracking logic"""
        try:
            logic = self._odmr_logic()
            logic.sigFitCompleted.disconnect(self._update_fit_display)
            logic.sigFitFailed.disconnect(self._on_fit_failed)
            logic.sigStreamStateChanged.disconnect(self._update_stream_state)
            logic.sigLockStateChanged.disconnect(self._update_lock_state)
            logic.sigLockStatusUpdated.disconnect(self._update_lock_status)
            logic.sigErrorDataUpdated.disconnect(self._update_error_plot)
            logic.sigStreamModeChanged.disconnect(self._update_stream_mode_ui)

            self.sigDoFit.disconnect()
            self.sigConfigureLock.disconnect()
            self.sigClearIntegrator.disconnect()
            self.sigSetStreamEnabled.disconnect()
            self.sigSetStreamMode.disconnect()
            self.sigSetLockEnabled.disconnect()
        except (TypeError, RuntimeError):
            pass

    # =========================================================================
    # GUI Slots (User Interactions)
    # =========================================================================

    @QtCore.Slot()
    def _do_fit(self):
        """Trigger resonance fit"""
        freq_min = self._freq_min_spinbox.value()
        freq_max = self._freq_max_spinbox.value()

        if freq_min >= freq_max:
            QtWidgets.QMessageBox.warning(
                self._mw,
                'Invalid Range',
                'Minimum frequency must be less than maximum frequency.'
            )
            return

        # Store values
        self._fit_freq_min = freq_min
        self._fit_freq_max = freq_max

        # Update fit region visual
        self._fit_region.setRegion([freq_min, freq_max])

        # Emit signal to logic
        self.sigDoFit.emit(freq_min, freq_max)

    @QtCore.Slot()
    def _on_fit_region_changed(self):
        """Handle fit region dragged by user"""
        region = self._fit_region.getRegion()
        self._freq_min_spinbox.blockSignals(True)
        self._freq_max_spinbox.blockSignals(True)
        self._freq_min_spinbox.setValue(region[0])
        self._freq_max_spinbox.setValue(region[1])
        self._freq_min_spinbox.blockSignals(False)
        self._freq_max_spinbox.blockSignals(False)

    @QtCore.Slot()
    def _update_fit_region_from_spinboxes(self):
        """Update fit region visualization when spinbox values change"""
        freq_min = self._freq_min_spinbox.value()
        freq_max = self._freq_max_spinbox.value()
        
        # Only update if values are valid
        if freq_min < freq_max:
            self._fit_region.blockSignals(True)
            self._fit_region.setRegion([freq_min, freq_max])
            self._fit_region.blockSignals(False)

    @QtCore.Slot()
    def _configure_lock(self):
        """Configure frequency lock"""
        mode = 'pi' if self._pi_radio.isChecked() else 'integral'
        bandwidth = self._bandwidth_spinbox.value()
        zero_ratio = self._zero_ratio_spinbox.value()

        # Store values
        self._lock_mode = mode
        self._lock_bandwidth = bandwidth
        self._lock_zero_ratio = zero_ratio

        # Emit signal
        self.sigConfigureLock.emit(mode, bandwidth, zero_ratio)

        # Enable lock controls
        self._enable_button.setEnabled(True)
        self._clear_button.setEnabled(True)

    @QtCore.Slot(str, float, float)
    def _handle_configure_lock(self, mode, bandwidth, zero_ratio):
        """Handle lock configuration in logic thread"""
        logic = self._odmr_logic()
        try:
            if mode == 'pi':
                logic.configure_lock_pi(bandwidth, zero_ratio=zero_ratio)
            else:
                logic.configure_lock(bandwidth_hz=bandwidth)
            self.log.info(
                f'Lock configured: mode={mode}, bandwidth={bandwidth} Hz' +
                (f', α={zero_ratio:.1f}' if mode == 'pi' else '')
            )
        except Exception as e:
            self.log.error(f'Failed to configure lock: {e}')

    @QtCore.Slot()
    def _clear_integrator(self):
        """Clear lock integrator"""
        self.sigClearIntegrator.emit()

    @QtCore.Slot(bool)
    def _set_stream_enabled(self, enable):
        """Enable or disable error signal streaming"""
        self.sigSetStreamEnabled.emit(enable)

    @QtCore.Slot(bool)
    def _handle_set_stream_enabled(self, enable):
        """Handle stream enable/disable in logic thread"""
        logic = self._odmr_logic()
        try:
            if enable:
                logic.start_error_stream()
            else:
                logic.stop_error_stream()
        except Exception as e:
            self.log.error(f'Failed to {"start" if enable else "stop"} error stream: {e}')

    @QtCore.Slot(bool)
    def _set_lock_enabled(self, enable):
        """Enable or disable tracking lock"""
        self.sigSetLockEnabled.emit(enable)

    @QtCore.Slot(bool)
    def _handle_set_lock_enabled(self, enable):
        """Handle lock enable/disable in logic thread"""
        logic = self._odmr_logic()
        try:
            if enable:
                logic.start_tracking()
            else:
                logic.stop_tracking()
        except Exception as e:
            self.log.error(f'Failed to {"enable" if enable else "disable"} lock: {e}')

    @QtCore.Slot()
    def _on_lock_mode_changed(self):
        """Handle lock mode radio button change"""
        is_pi = self._pi_radio.isChecked()
        self._zero_ratio_spinbox.setEnabled(is_pi)

    @QtCore.Slot()
    def _on_stream_mode_changed(self):
        """Handle stream mode radio button change"""
        is_error = self._error_mode_radio.isChecked()
        mode = 'error' if is_error else 'correction'

        # Update plot label
        if is_error:
            self._error_plot_widget.setLabel('left', 'Error Signal', units='LSB')
        else:
            self._error_plot_widget.setLabel('left', 'Frequency Correction', units='Hz')

        # Store selection
        self._stream_mode = mode

        # Emit signal to logic (will check if stream is stopped)
        self.sigSetStreamMode.emit(mode)

    # =========================================================================
    # Logic Slots (Updates from Logic)
    # =========================================================================

    @QtCore.Slot(dict)
    def _update_fit_display(self, fit_result):
        """Update fit display with results"""
        try:
            slope = fit_result.get('slope')
            r_squared = fit_result.get('r_squared')
            fit_freq = fit_result.get('fit_frequency')
            fit_data = fit_result.get('fit_data')

            # Update labels
            if slope is not None:
                self._fit_slope_label.setText(f'{slope:.3e}')
            if r_squared is not None:
                self._fit_r2_label.setText(f'{r_squared:.4f}')

            # Update fit curve overlay on ODMR plot
            if fit_freq is not None and fit_data is not None:
                self._tracking_fit_curve.setData(x=fit_freq, y=fit_data)

            self.log.info(f'Fit completed: slope={slope:.3e}, R²={r_squared:.4f}')

        except Exception as e:
            self.log.error(f'Error updating fit display: {e}')

    @QtCore.Slot(str)
    def _on_fit_failed(self, error_msg):
        """Handle fit failure"""
        self.log.warning(f'Fit failed: {error_msg}')
        QtWidgets.QMessageBox.warning(
            self._mw,
            'Fit Failed',
            f'Resonance fitting failed:\n{error_msg}'
        )

    @QtCore.Slot(bool)
    def _update_stream_state(self, streaming):
        """Update stream state indicators"""
        if streaming:
            self._start_stream_button.setEnabled(False)
            self._stop_stream_button.setEnabled(True)
            # Disable mode selection while streaming
            self._error_mode_radio.setEnabled(False)
            self._correction_mode_radio.setEnabled(False)
        else:
            self._start_stream_button.setEnabled(True)
            self._stop_stream_button.setEnabled(False)
            # Re-enable mode selection
            self._error_mode_radio.setEnabled(True)
            self._correction_mode_radio.setEnabled(True)

    @QtCore.Slot(bool)
    def _update_lock_state(self, locked):
        """Update lock state indicators"""
        if locked:
            self._status_labels['locked'].setText('ON')
            self._status_labels['locked'].setStyleSheet('QLabel { color: green; font-weight: bold; }')
            self._enable_button.setEnabled(False)
            self._disable_button.setEnabled(True)
            self._configure_button.setEnabled(False)
        else:
            self._status_labels['locked'].setText('OFF')
            self._status_labels['locked'].setStyleSheet('QLabel { color: red; }')
            self._enable_button.setEnabled(True)
            self._disable_button.setEnabled(False)
            self._configure_button.setEnabled(True)

    @QtCore.Slot(dict)
    def _update_lock_status(self, status):
        """Update lock status indicators"""
        try:
            saturated = status.get('saturated', False)
            error_lsb = status.get('error_lsb', 0)
            correction_hz = status.get('correction_hz', 0)

            # Update status labels
            self._status_labels['saturated'].setText('YES' if saturated else 'NO')
            if saturated:
                self._status_labels['saturated'].setStyleSheet('QLabel { color: red; font-weight: bold; }')
            else:
                self._status_labels['saturated'].setStyleSheet('QLabel { color: green; }')

            self._status_labels['error'].setText(f'{error_lsb:.1f}')
            self._status_labels['correction'].setText(f'{correction_hz:.3e}')

        except Exception as e:
            self.log.error(f'Error updating lock status: {e}')

    @QtCore.Slot(object, object)
    def _update_error_plot(self, times, error_data):
        """Update error signal time series plot"""
        try:
            if times is not None and error_data is not None:
                # Only show non-zero portion of circular buffer
                if len(times) > 0 and len(error_data) > 0:
                    # Shift time axis to show relative time
                    if times[-1] > 0:
                        rel_times = times - times[-1]
                        self._error_curve.setData(x=rel_times, y=error_data)
        except Exception as e:
            self.log.error(f'Error updating error plot: {e}')

    @QtCore.Slot(str)
    def _handle_set_stream_mode(self, mode):
        """Handle stream mode change in logic thread"""
        logic = self._odmr_logic()
        try:
            logic.set_stream_mode(mode)
        except RuntimeError as e:
            # Stream is active - warn user
            self.log.warning(f'Cannot change mode while streaming: {e}')
            QtWidgets.QMessageBox.warning(
                self._mw,
                'Stream Active',
                'Cannot change stream mode while streaming is active. Stop stream first.'
            )
            # Revert radio button to current mode
            self._update_stream_mode_ui(logic.stream_mode)
        except Exception as e:
            self.log.error(f'Failed to set stream mode: {e}')

    @QtCore.Slot(str)
    def _update_stream_mode_ui(self, mode: str):
        """Update radio buttons to match logic state"""
        self._error_mode_radio.blockSignals(True)
        self._correction_mode_radio.blockSignals(True)

        if mode == 'error':
            self._error_mode_radio.setChecked(True)
            self._error_plot_widget.setLabel('left', 'Error Signal', units='LSB')
        else:
            self._correction_mode_radio.setChecked(True)
            self._error_plot_widget.setLabel('left', 'Frequency Correction', units='Hz')

        self._error_mode_radio.blockSignals(False)
        self._correction_mode_radio.blockSignals(False)

    # =========================================================================
    # Helper Methods
    # =========================================================================

    @QtCore.Slot()
    def restore_default_view(self):
        """Override parent's restore_default_view to include tracking docks"""
        # Call parent to restore ODMR docks
        super().restore_default_view()
        
        # Add tracking-specific docks
        if self._fit_controls_dock is not None:
            self._fit_controls_dock.setFloating(False)
            self._mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, self._fit_controls_dock)
            
        if self._lock_controls_dock is not None:
            self._lock_controls_dock.setFloating(False)
            self._mw.addDockWidget(QtCore.Qt.RightDockWidgetArea, self._lock_controls_dock)
            # Tabify lock controls with fit controls
            if self._fit_controls_dock is not None:
                self._mw.tabifyDockWidget(self._fit_controls_dock, self._lock_controls_dock)
            
        if self._time_series_dock is not None:
            self._time_series_dock.setFloating(False)
            self._mw.addDockWidget(QtCore.Qt.BottomDockWidgetArea, self._time_series_dock)
        
        # Ensure all dock widgets are visible
        if self._fit_controls_dock is not None:
            self._fit_controls_dock.show()
        if self._lock_controls_dock is not None:
            self._lock_controls_dock.show()
        if self._time_series_dock is not None:
            self._time_series_dock.show()

    def _restore_tracking_settings(self):
        """Restore saved tracking settings to GUI"""
        # Restore fit range
        self._freq_min_spinbox.setValue(self._fit_freq_min)
        self._freq_max_spinbox.setValue(self._fit_freq_max)
        self._fit_region.setRegion([self._fit_freq_min, self._fit_freq_max])

        # Restore lock parameters
        self._bandwidth_spinbox.setValue(self._lock_bandwidth)
        self._zero_ratio_spinbox.setValue(self._lock_zero_ratio)

        # Restore lock mode
        if self._lock_mode == 'pi':
            self._pi_radio.setChecked(True)
            self._zero_ratio_spinbox.setEnabled(True)
        else:
            self._integral_radio.setChecked(True)
            self._zero_ratio_spinbox.setEnabled(False)

        # Restore stream mode
        self._update_stream_mode_ui(self._stream_mode)
