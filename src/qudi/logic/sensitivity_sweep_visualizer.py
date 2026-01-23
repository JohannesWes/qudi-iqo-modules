# -*- coding: utf-8 -*-
"""
Sensitivity Sweep Visualization Module

Generates summary plots from sensitivity parameter sweep results, showing how
sensitivity and ODMR parameters vary with the swept parameters (power_dbm,
f_mod_hz, f_dev_khz).

This module is designed to be called from sensitivity_sweep_logic.py but is
kept separate for maintainability and to avoid bloating the main logic module.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level
directory of this distribution and on <https://github.com/Ulm-IQO/qudi-core/>
"""

__all__ = ['SensitivitySweepVisualizer']

import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from typing import Dict, Any, Optional, List, Tuple
import logging

# Use non-interactive backend for file saving (avoids Qt conflicts in threaded context)
matplotlib.use('Agg')


class SensitivitySweepVisualizer:
    """
    Generates summary visualization plots for sensitivity parameter sweeps.

    This class takes the results DataFrame and metadata from a sensitivity sweep
    and produces a set of plots showing how sensitivity and ODMR parameters
    vary with the swept parameters.

    Usage:
        visualizer = SensitivitySweepVisualizer(results_df, metadata, output_folder)
        visualizer.generate_all_plots()
    """

    def __init__(self, results_df: pd.DataFrame, metadata: Dict[str, Any],
                 output_folder: str, logger: Optional[logging.Logger] = None):
        """
        Initialize the visualizer.

        Args:
            results_df: DataFrame with sweep results (from parameter_sweep_summary.csv)
            metadata: Dictionary with sweep metadata (from sweep_metadata.json)
            output_folder: Folder where plots will be saved
            logger: Optional logger instance (uses module logger if not provided)
        """
        self.df = results_df
        self.metadata = metadata
        self.output_folder = output_folder
        self.log = logger or logging.getLogger(__name__)

        # Create summary_plots subfolder
        self.plots_folder = os.path.join(output_folder, 'summary_plots')
        if not os.path.exists(self.plots_folder):
            os.makedirs(self.plots_folder)

        # Extract sweep loop order
        self.loop_order = metadata.get('sweep_loop_order', ['power', 'f_mod', 'f_dev'])

        # Determine which parameters were actually varied (more than 1 unique value)
        self.varied_params = self._identify_varied_parameters()

    def _identify_varied_parameters(self) -> Dict[str, np.ndarray]:
        """Identify which parameters were varied in the sweep."""
        varied = {}
        param_columns = {
            'power': 'power_dbm',
            'f_mod': 'f_mod_hz',
            'f_dev': 'f_dev_khz'
        }

        for param_key, col_name in param_columns.items():
            if col_name in self.df.columns:
                unique_vals = self.df[col_name].unique()
                if len(unique_vals) > 1:
                    varied[param_key] = np.sort(unique_vals)

        return varied

    def generate_all_plots(self) -> List[str]:
        """
        Generate all summary plots.

        Returns:
            List of paths to generated plot files
        """
        generated_files = []

        if len(self.df) == 0:
            self.log.warning('No data to visualize - results DataFrame is empty')
            return generated_files

        # Only generate plots if we have at least 2 data points
        if len(self.df) < 2:
            self.log.warning('Insufficient data for visualization (need at least 2 points)')
            return generated_files

        try:
            # 1. Sensitivity heatmap (if 2 parameters varied)
            if len(self.varied_params) >= 2:
                path = self._plot_sensitivity_heatmap()
                if path:
                    generated_files.append(path)

            # 2. Sensitivity vs each varied parameter (line plots)
            for param_key in self.varied_params:
                path = self._plot_sensitivity_vs_parameter(param_key)
                if path:
                    generated_files.append(path)

            # 3. ODMR center vs parameters (if varied)
            if len(self.varied_params) >= 1:
                path = self._plot_odmr_center_vs_parameters()
                if path:
                    generated_files.append(path)

            # 4. Linewidth vs parameters
            if 'linewidth_hz' in self.df.columns and len(self.varied_params) >= 1:
                path = self._plot_linewidth_vs_parameters()
                if path:
                    generated_files.append(path)

            # 5. Summary statistics plot
            path = self._plot_summary_statistics()
            if path:
                generated_files.append(path)

            self.log.info(f'Generated {len(generated_files)} summary plots in {self.plots_folder}')

        except Exception as e:
            self.log.error(f'Error generating plots: {e}', exc_info=True)

        return generated_files

    def _plot_sensitivity_heatmap(self) -> Optional[str]:
        """
        Create 2D heatmap of sensitivity vs two varied parameters.

        Returns:
            Path to saved figure, or None if plot couldn't be generated
        """
        # Determine which two parameters to use for the heatmap
        # Prefer power and f_dev as they are most commonly varied together
        param_priority = ['power', 'f_dev', 'f_mod']
        x_param, y_param = None, None

        for param in param_priority:
            if param in self.varied_params:
                if x_param is None:
                    x_param = param
                elif y_param is None:
                    y_param = param
                    break

        if x_param is None or y_param is None:
            self.log.debug('Not enough varied parameters for heatmap')
            return None

        # Map parameter keys to column names
        col_map = {'power': 'power_dbm', 'f_mod': 'f_mod_hz', 'f_dev': 'f_dev_khz'}
        x_col, y_col = col_map[x_param], col_map[y_param]

        # Get unique values
        x_vals = np.sort(self.df[x_col].unique())
        y_vals = np.sort(self.df[y_col].unique())

        # Create sensitivity matrix
        sens_matrix = np.full((len(y_vals), len(x_vals)), np.nan)

        for i, y_val in enumerate(y_vals):
            for j, x_val in enumerate(x_vals):
                mask = (self.df[x_col] == x_val) & (self.df[y_col] == y_val)
                if mask.any():
                    sens_matrix[i, j] = self.df.loc[mask, 'sensitivity_nT_rtHz'].values[0]

        # Find best point
        best_idx = self.df['sensitivity_nT_rtHz'].idxmin()
        best_x = self.df.loc[best_idx, x_col]
        best_y = self.df.loc[best_idx, y_col]
        best_sens = self.df.loc[best_idx, 'sensitivity_nT_rtHz']

        # Create figure
        fig, ax = plt.subplots(figsize=(10, 8))

        # Use appropriate normalization based on data range
        valid_sens = sens_matrix[~np.isnan(sens_matrix)]
        if len(valid_sens) > 0 and valid_sens.min() > 0:
            vmin, vmax = valid_sens.min() * 0.9, valid_sens.max() * 1.1
            norm = None  # Linear scale for typical sensitivity values
        else:
            vmin, vmax = None, None
            norm = None

        # Plot heatmap
        im = ax.imshow(sens_matrix, aspect='auto', origin='lower',
                       extent=[x_vals.min(), x_vals.max(), y_vals.min(), y_vals.max()],
                       cmap='viridis_r', vmin=vmin, vmax=vmax)

        # Add colorbar
        cbar = fig.colorbar(im, ax=ax, label=r'Sensitivity [nT/$\sqrt{\mathrm{Hz}}$]')

        # Mark best point
        ax.scatter([best_x], [best_y], marker='*', s=300, c='red', edgecolors='white',
                   linewidths=2, zorder=5, label=f'Best: {best_sens:.3f} nT/√Hz')

        # Labels
        x_label_map = {'power_dbm': 'Power [dBm]', 'f_mod_hz': 'Modulation Frequency [Hz]',
                       'f_dev_khz': 'FM Deviation [kHz]'}
        ax.set_xlabel(x_label_map.get(x_col, x_col))
        ax.set_ylabel(x_label_map.get(y_col, y_col))
        ax.set_title('Magnetic Field Sensitivity vs Parameters')
        ax.legend(loc='upper right')

        # Add grid lines at parameter values
        ax.set_xticks(x_vals)
        ax.set_yticks(y_vals)
        ax.grid(True, alpha=0.3, linestyle='--')

        fig.tight_layout()

        # Save
        filepath = os.path.join(self.plots_folder, 'sensitivity_heatmap.pdf')
        fig.savefig(filepath, dpi=150)
        plt.close(fig)

        self.log.debug(f'Saved sensitivity heatmap: {filepath}')
        return filepath

    def _plot_sensitivity_vs_parameter(self, param_key: str) -> Optional[str]:
        """
        Create line plot of sensitivity vs one parameter, with lines for each
        value of the other varied parameter.

        Args:
            param_key: Parameter key ('power', 'f_mod', or 'f_dev')

        Returns:
            Path to saved figure, or None if plot couldn't be generated
        """
        col_map = {'power': 'power_dbm', 'f_mod': 'f_mod_hz', 'f_dev': 'f_dev_khz'}
        x_col = col_map[param_key]

        if x_col not in self.df.columns:
            return None

        # Determine grouping parameter (the other varied parameter)
        other_params = [p for p in self.varied_params if p != param_key]
        group_param = other_params[0] if other_params else None
        group_col = col_map.get(group_param) if group_param else None

        fig, ax = plt.subplots(figsize=(10, 6))

        if group_col and group_col in self.df.columns:
            # Plot lines for each group value
            group_vals = np.sort(self.df[group_col].unique())
            colors = plt.cm.viridis(np.linspace(0, 0.9, len(group_vals)))

            for color, group_val in zip(colors, group_vals):
                mask = self.df[group_col] == group_val
                subset = self.df[mask].sort_values(x_col)

                # Format label based on parameter type
                if group_param == 'power':
                    label = f'{group_val:.1f} dBm'
                elif group_param == 'f_mod':
                    label = f'{group_val/1e3:.1f} kHz'
                elif group_param == 'f_dev':
                    label = f'{group_val:.1f} kHz'
                else:
                    label = str(group_val)

                ax.plot(subset[x_col], subset['sensitivity_nT_rtHz'],
                        'o-', color=color, label=label, markersize=8, linewidth=2)
        else:
            # Single line if no grouping
            sorted_df = self.df.sort_values(x_col)
            ax.plot(sorted_df[x_col], sorted_df['sensitivity_nT_rtHz'],
                    'o-', markersize=8, linewidth=2)

        # Mark best point
        best_idx = self.df['sensitivity_nT_rtHz'].idxmin()
        best_x = self.df.loc[best_idx, x_col]
        best_sens = self.df.loc[best_idx, 'sensitivity_nT_rtHz']
        ax.scatter([best_x], [best_sens], marker='*', s=300, c='red', edgecolors='white',
                   linewidths=2, zorder=10, label=f'Best: {best_sens:.3f}')

        # Labels and formatting
        x_label_map = {'power_dbm': 'Power [dBm]', 'f_mod_hz': 'Modulation Frequency [Hz]',
                       'f_dev_khz': 'FM Deviation [kHz]'}
        ax.set_xlabel(x_label_map.get(x_col, x_col))
        ax.set_ylabel(r'Sensitivity [nT/$\sqrt{\mathrm{Hz}}$]')

        title_map = {'power_dbm': 'Power', 'f_mod_hz': 'Modulation Frequency',
                     'f_dev_khz': 'FM Deviation'}
        ax.set_title(f'Sensitivity vs {title_map.get(x_col, x_col)}')

        ax.grid(True, alpha=0.3)
        ax.legend(title=x_label_map.get(group_col, '') if group_col else None,
                  loc='best', fontsize=9)

        fig.tight_layout()

        # Save
        filepath = os.path.join(self.plots_folder, f'sensitivity_vs_{param_key}.pdf')
        fig.savefig(filepath, dpi=150)
        plt.close(fig)

        self.log.debug(f'Saved sensitivity vs {param_key}: {filepath}')
        return filepath

    def _plot_odmr_center_vs_parameters(self) -> Optional[str]:
        """
        Plot ODMR center frequency vs swept parameters.

        Returns:
            Path to saved figure, or None if plot couldn't be generated
        """
        if 'odmr_center_hz' not in self.df.columns:
            return None

        col_map = {'power': 'power_dbm', 'f_mod': 'f_mod_hz', 'f_dev': 'f_dev_khz'}

        # Create subplots for each varied parameter
        n_params = len(self.varied_params)
        if n_params == 0:
            return None

        fig, axes = plt.subplots(1, n_params, figsize=(5 * n_params, 5))
        if n_params == 1:
            axes = [axes]

        for ax, param_key in zip(axes, self.varied_params):
            x_col = col_map[param_key]

            # Group by this parameter and calculate mean ODMR center
            grouped = self.df.groupby(x_col)['odmr_center_hz'].agg(['mean', 'std'])
            x_vals = grouped.index.values
            y_vals = grouped['mean'].values / 1e6  # Convert to MHz
            y_err = grouped['std'].values / 1e3  # Convert to kHz for error bars

            ax.errorbar(x_vals, y_vals, yerr=y_err / 1e3, fmt='o-',
                        capsize=4, markersize=8, linewidth=2)

            # Labels
            x_label_map = {'power_dbm': 'Power [dBm]', 'f_mod_hz': 'Mod. Freq. [Hz]',
                           'f_dev_khz': 'FM Deviation [kHz]'}
            ax.set_xlabel(x_label_map.get(x_col, x_col))
            ax.set_ylabel('ODMR Center [MHz]')
            ax.grid(True, alpha=0.3)

        fig.suptitle('ODMR Center Frequency vs Parameters', fontsize=12)
        fig.tight_layout()

        # Save
        filepath = os.path.join(self.plots_folder, 'odmr_center_vs_parameters.pdf')
        fig.savefig(filepath, dpi=150)
        plt.close(fig)

        self.log.debug(f'Saved ODMR center vs parameters: {filepath}')
        return filepath

    def _plot_linewidth_vs_parameters(self) -> Optional[str]:
        """
        Plot ODMR linewidth vs swept parameters.

        Returns:
            Path to saved figure, or None if plot couldn't be generated
        """
        if 'linewidth_hz' not in self.df.columns:
            return None

        col_map = {'power': 'power_dbm', 'f_mod': 'f_mod_hz', 'f_dev': 'f_dev_khz'}

        n_params = len(self.varied_params)
        if n_params == 0:
            return None

        fig, axes = plt.subplots(1, n_params, figsize=(5 * n_params, 5))
        if n_params == 1:
            axes = [axes]

        for ax, param_key in zip(axes, self.varied_params):
            x_col = col_map[param_key]

            # Group by this parameter
            grouped = self.df.groupby(x_col)['linewidth_hz'].agg(['mean', 'std'])
            x_vals = grouped.index.values
            y_vals = grouped['mean'].values / 1e3  # Convert to kHz
            y_err = grouped['std'].values / 1e3

            ax.errorbar(x_vals, y_vals, yerr=y_err, fmt='o-',
                        capsize=4, markersize=8, linewidth=2, color='orange')

            # Labels
            x_label_map = {'power_dbm': 'Power [dBm]', 'f_mod_hz': 'Mod. Freq. [Hz]',
                           'f_dev_khz': 'FM Deviation [kHz]'}
            ax.set_xlabel(x_label_map.get(x_col, x_col))
            ax.set_ylabel('Linewidth [kHz]')
            ax.grid(True, alpha=0.3)

        fig.suptitle('ODMR Linewidth vs Parameters', fontsize=12)
        fig.tight_layout()

        # Save
        filepath = os.path.join(self.plots_folder, 'linewidth_vs_parameters.pdf')
        fig.savefig(filepath, dpi=150)
        plt.close(fig)

        self.log.debug(f'Saved linewidth vs parameters: {filepath}')
        return filepath

    def _plot_summary_statistics(self) -> Optional[str]:
        """
        Create a summary statistics plot with key metrics.

        Returns:
            Path to saved figure, or None if plot couldn't be generated
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # Get valid sensitivity data
        sens = self.df['sensitivity_nT_rtHz'].values
        valid_sens = sens[~np.isnan(sens)]

        if len(valid_sens) == 0:
            plt.close(fig)
            return None

        # Top-left: Sensitivity histogram
        ax = axes[0, 0]
        ax.hist(valid_sens, bins=min(20, len(valid_sens)), edgecolor='black', alpha=0.7)
        ax.axvline(np.min(valid_sens), color='g', linestyle='--',
                   label=f'Best: {np.min(valid_sens):.3f}')
        ax.axvline(np.mean(valid_sens), color='r', linestyle='--',
                   label=f'Mean: {np.mean(valid_sens):.3f}')
        ax.set_xlabel(r'Sensitivity [nT/$\sqrt{\mathrm{Hz}}$]')
        ax.set_ylabel('Count')
        ax.set_title('Sensitivity Distribution')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Top-right: Sensitivity vs measurement index
        ax = axes[0, 1]
        indices = np.arange(len(sens))
        ax.plot(indices, sens, 'o-', markersize=6, alpha=0.7)
        ax.axhline(np.nanmin(sens), color='g', linestyle='--', alpha=0.7)
        ax.set_xlabel('Measurement Index')
        ax.set_ylabel(r'Sensitivity [nT/$\sqrt{\mathrm{Hz}}$]')
        ax.set_title('Sensitivity vs Measurement Order')
        ax.grid(True, alpha=0.3)

        # Bottom-left: Summary text
        ax = axes[1, 0]
        ax.axis('off')

        # Compile statistics
        best_params = self.metadata.get('best_parameters', {})
        stats_text = [
            f"Sweep Summary",
            f"─" * 40,
            f"Total measurements: {len(self.df)}",
            f"Valid measurements: {len(valid_sens)}",
            f"",
            f"Sensitivity Statistics:",
            f"  Best:   {np.min(valid_sens):.4f} nT/√Hz",
            f"  Mean:   {np.mean(valid_sens):.4f} nT/√Hz",
            f"  Median: {np.median(valid_sens):.4f} nT/√Hz",
            f"  Std:    {np.std(valid_sens):.4f} nT/√Hz",
            f"",
            f"Best Parameters:",
        ]

        # Format best parameters
        for key, val in best_params.items():
            if key == 'power':
                stats_text.append(f"  Power:    {val:.2f} dBm")
            elif key == 'f_mod':
                stats_text.append(f"  f_mod:    {val/1e3:.2f} kHz")
            elif key == 'f_dev':
                stats_text.append(f"  f_dev:    {val:.1f} kHz")

        # ODMR center stats if available
        if 'odmr_center_hz' in self.df.columns:
            odmr_valid = self.df['odmr_center_hz'].dropna()
            if len(odmr_valid) > 0:
                stats_text.extend([
                    f"",
                    f"ODMR Center Stability:",
                    f"  Mean:  {odmr_valid.mean()/1e9:.6f} GHz",
                    f"  Drift: {(odmr_valid.max() - odmr_valid.min())/1e3:.1f} kHz",
                ])

        ax.text(0.1, 0.95, '\n'.join(stats_text), transform=ax.transAxes,
                fontsize=11, verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Bottom-right: Box plot of sensitivity by parameter
        ax = axes[1, 1]

        # Create box plot for one of the varied parameters
        if self.varied_params:
            param_key = list(self.varied_params.keys())[0]
            col_map = {'power': 'power_dbm', 'f_mod': 'f_mod_hz', 'f_dev': 'f_dev_khz'}
            x_col = col_map[param_key]

            # Group data
            groups = []
            labels = []
            for val in self.varied_params[param_key]:
                mask = self.df[x_col] == val
                group_sens = self.df.loc[mask, 'sensitivity_nT_rtHz'].dropna().values
                if len(group_sens) > 0:
                    groups.append(group_sens)
                    if param_key == 'power':
                        labels.append(f'{val:.1f}')
                    elif param_key == 'f_mod':
                        labels.append(f'{val/1e3:.1f}')
                    else:
                        labels.append(f'{val:.0f}')

            if groups:
                ax.boxplot(groups, labels=labels)
                label_map = {'power': 'Power [dBm]', 'f_mod': 'f_mod [kHz]', 'f_dev': 'f_dev [kHz]'}
                ax.set_xlabel(label_map.get(param_key, param_key))
                ax.set_ylabel(r'Sensitivity [nT/$\sqrt{\mathrm{Hz}}$]')
                ax.set_title(f'Sensitivity by {param_key}')
                ax.grid(True, alpha=0.3, axis='y')
        else:
            ax.text(0.5, 0.5, 'Single parameter point', ha='center', va='center',
                    transform=ax.transAxes)
            ax.axis('off')

        fig.tight_layout()

        # Save
        filepath = os.path.join(self.plots_folder, 'sweep_summary.pdf')
        fig.savefig(filepath, dpi=150)
        plt.close(fig)

        self.log.debug(f'Saved summary statistics: {filepath}')
        return filepath


def generate_sweep_visualizations(results_csv_path: str, metadata_json_path: str,
                                   output_folder: Optional[str] = None,
                                   logger: Optional[logging.Logger] = None) -> List[str]:
    """
    Convenience function to generate all visualizations from file paths.

    Args:
        results_csv_path: Path to parameter_sweep_summary.csv
        metadata_json_path: Path to sweep_metadata.json
        output_folder: Optional output folder (defaults to same as CSV file)
        logger: Optional logger instance

    Returns:
        List of paths to generated plot files
    """
    import json

    # Load data
    df = pd.read_csv(results_csv_path, sep='\t')

    with open(metadata_json_path, 'r') as f:
        metadata = json.load(f)

    # Determine output folder
    if output_folder is None:
        output_folder = os.path.dirname(results_csv_path)

    # Create visualizer and generate plots
    visualizer = SensitivitySweepVisualizer(df, metadata, output_folder, logger)
    return visualizer.generate_all_plots()
