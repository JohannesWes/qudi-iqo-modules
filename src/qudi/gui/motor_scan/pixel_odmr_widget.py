# -*- coding: utf-8 -*-
"""
Widget for displaying ODMR spectrum of a selected pixel in motor XY scan.

Copyright (c) 2024, the qudi developers.
"""

import numpy as np
from typing import Optional, Dict, Any
from PySide2 import QtCore, QtWidgets
import pyqtgraph as pg

from qudi.util.colordefs import QudiPalettePale as palette


class PixelOdmrSpectrumWidget(QtWidgets.QWidget):
    """
    Widget displaying ODMR spectrum for a selected pixel in motor scan.

    Shows the raw ODMR signal vs frequency with optional fit markers
    at the zero-crossing frequencies.
    """

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)

        # State
        self._has_data = False
        self._resonance_lines = []

        # Create layout
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Info label at top
        self._info_label = QtWidgets.QLabel('Click a pixel to view ODMR spectrum')
        self._info_label.setWordWrap(True)
        self._info_label.setStyleSheet('font-size: 10pt; padding: 4px;')
        layout.addWidget(self._info_label)

        # Plot widget
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setBackground('w')
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setLabel('bottom', 'Frequency', units='Hz')
        self._plot_widget.setLabel('left', 'Signal', units='V')
        layout.addWidget(self._plot_widget, stretch=1)

        # Create plot data items
        self._data_curve = pg.PlotDataItem(
            pen=pg.mkPen(color=palette.c1, width=1),
            symbol='o',
            symbolSize=4,
            symbolBrush=palette.c1,
            symbolPen=None
        )
        self._plot_widget.addItem(self._data_curve)

        # Legend (will add items dynamically)
        self._legend = self._plot_widget.addLegend(offset=(10, 10))
        self._legend.addItem(self._data_curve, 'ODMR Signal')

    def set_pixel_data(
        self,
        frequency_data: Optional[np.ndarray],
        signal_data: Optional[Dict[str, np.ndarray]],
        fit_result: Optional[Dict[str, Any]],
        pixel_info: Dict[str, Any]
    ) -> None:
        """
        Update display with ODMR data for a selected pixel.

        Args:
            frequency_data: 1D array of frequencies in Hz
            signal_data: Dict mapping channel name to 1D signal array
            fit_result: Dict with fit results (zero_crossing_frequencies, linewidths, etc.)
            pixel_info: Dict with 'grid_index' and 'position_mm'
        """
        # Clear previous resonance markers
        self._clear_resonance_markers()

        # Validate input data
        if frequency_data is None or signal_data is None:
            self.clear()
            self._info_label.setText('No ODMR data for this pixel')
            return

        # Get the first channel's signal data
        if isinstance(signal_data, dict):
            if len(signal_data) == 0:
                self.clear()
                self._info_label.setText('No signal data available')
                return
            channel_name = list(signal_data.keys())[0]
            signal = signal_data[channel_name]
        else:
            signal = signal_data

        # Ensure arrays are proper numpy arrays
        freq = np.asarray(frequency_data)
        sig = np.asarray(signal)

        if len(freq) == 0 or len(sig) == 0:
            self.clear()
            self._info_label.setText('Empty data for this pixel')
            return

        # Update plot data
        self._data_curve.setData(freq, sig)
        self._has_data = True

        # Auto-range to show all data
        self._plot_widget.autoRange()

        # Build info text
        ix, iy = pixel_info.get('grid_index', (0, 0))
        x_mm, y_mm = pixel_info.get('position_mm', (0.0, 0.0))
        info_text = f'Pixel ({ix}, {iy}) at x={x_mm:.3f}mm, y={y_mm:.3f}mm'

        # Add fit information and markers if available
        if fit_result is not None:
            zc_freqs = fit_result.get('zero_crossing_frequencies [Hz]')
            linewidths = fit_result.get('linewidths [Hz]')
            n_features = fit_result.get('n_features_found', 0)

            # Add resonance markers
            if zc_freqs is not None and len(zc_freqs) > 0:
                zc_freqs = np.asarray(zc_freqs)
                valid_zc = zc_freqs[~np.isnan(zc_freqs)]

                for freq_val in valid_zc:
                    line = pg.InfiniteLine(
                        pos=freq_val,
                        angle=90,
                        pen=pg.mkPen(color='r', width=1.5, style=QtCore.Qt.DashLine),
                        label=f'{freq_val/1e9:.4f} GHz',
                        labelOpts={'position': 0.9, 'color': 'r', 'fill': (255, 255, 255, 150)}
                    )
                    self._plot_widget.addItem(line)
                    self._resonance_lines.append(line)

                # Add center frequency to info
                if len(valid_zc) > 0:
                    center_freq = np.mean(valid_zc)
                    info_text += f' | f0={center_freq/1e9:.4f} GHz'

            # Add linewidth to info
            if linewidths is not None and len(linewidths) > 0:
                linewidths = np.asarray(linewidths)
                valid_lw = linewidths[~np.isnan(linewidths)]
                if len(valid_lw) > 0:
                    mean_lw = np.mean(valid_lw)
                    info_text += f', dv={mean_lw/1e6:.2f} MHz'

            # Add feature count
            if n_features > 0:
                info_text += f' ({n_features} peaks)'

        self._info_label.setText(info_text)

    def clear(self) -> None:
        """Clear the display."""
        self._data_curve.setData([], [])
        self._clear_resonance_markers()
        self._has_data = False
        self._info_label.setText('Click a pixel to view ODMR spectrum')

    def _clear_resonance_markers(self) -> None:
        """Remove all resonance marker lines from the plot."""
        for line in self._resonance_lines:
            self._plot_widget.removeItem(line)
        self._resonance_lines.clear()

    @property
    def has_data(self) -> bool:
        """Whether the widget currently has data displayed."""
        return self._has_data
