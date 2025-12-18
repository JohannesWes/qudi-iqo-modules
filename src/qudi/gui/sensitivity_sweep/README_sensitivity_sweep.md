# Sensitivity Sweep Logic Module

## Overview

The `SensitivitySweepLogic` module provides automated ODMR-based magnetic field sensitivity measurements with systematic parameter sweeps. It orchestrates the complete measurement workflow without requiring external scripts or RPyC remote access, eliminating serialization overhead and enabling real-time GUI integration.

## Key Features

- **Automated Parameter Sweeps**: Systematically explore microwave power, FM modulation frequency, and FM deviation
- **Pausable/Resumable**: Full state persistence allows pausing and resuming long-running sweeps
- **Thermal Management**: Automatic stabilization delays when power changes
- **Real-Time Updates**: Qt signals provide live progress and data for GUI visualization
- **Robust Error Handling**: Failed measurements are logged and skipped without stopping the sweep
- **Zero Overhead**: Direct connector access to ODMR and time series modules (no RPyC serialization)
- **Flexible Loop Order**: Configure which parameter changes most frequently

## Architecture

### Measurement Workflow

For each parameter combination:

1. **Configure MW Source** → Set power, FM frequency, FM deviation via ODMR logic
2. **Thermal Stabilization** → Wait if power changed (configurable delay)
3. **ODMR Scan** → Run full frequency scan at configured parameters
4. **Fit Resonance** → Extract zero-crossing frequency and slope using `fit_hyperfine()`
5. **Set CW Frequency** → Configure CW output at zero-crossing
6. **Record Time Series** → Acquire N traces via time_series_reader_logic
7. **Calculate Sensitivity** → Convert voltage → B-field → ASD → sensitivity
8. **Save Results** → Store data and emit signals for GUI updates

### State Machine

```
idle ─────start_sweep()─────→ running ─────pause_sweep()─────→ paused
  ↑                              │  ↑                              │
  │                              │  │                              │
  └────────finish────────────────┘  └──────resume_sweep()──────────┘
  │
  └────────cancel_sweep()────────────────────┘ (from running or paused)
```

## Configuration

### Module Definition

```yaml
logic:
    sensitivity_sweep_logic:
        module.Class: 'sensitivity_sweep_logic.SensitivitySweepLogic'
        options:
            # Thermal stabilization delay when power changes (seconds)
            thermal_stabilization_time: 180

            # Loop order for parameter sweep
            # Outer loop (first element) changes least frequently
            # Options: 'power', 'f_mod', 'f_dev'
            sweep_loop_order: ['power', 'f_mod', 'f_dev']

            # ODMR fitting parameters
            which_zero_crossing: 2  # Index of zero crossing to use (0-indexed)
            n_most_prominent_peaks: 5
            min_fit_amplitude: 0.005
            min_feature_height: 0.003

            # Off-resonant measurement (optional)
            include_off_resonant_measurement: False
            off_resonant_offset_hz: 30e6  # 30 MHz offset
        connectors:
            odmr_logic: 'odmr_logic'
            time_series_logic: 'time_series_reader_logic'
```

### Config Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `thermal_stabilization_time` | int | 180 | Seconds to wait when power changes |
| `sweep_loop_order` | list | `['power', 'f_mod', 'f_dev']` | Parameter iteration order |
| `which_zero_crossing` | int | 2 | Zero-crossing index for CW frequency |
| `n_most_prominent_peaks` | int | 5 | Number of peaks to consider in fit |
| `min_fit_amplitude` | float | 0.005 | Minimum feature prominence for fitting |
| `min_feature_height` | float | 0.003 | Minimum peak height for fitting |
| `include_off_resonant_measurement` | bool | False | Measure off-resonant sensitivity |
| `off_resonant_offset_hz` | float | 30e6 | Frequency offset for off-resonant measurement |

### Loop Order

The `sweep_loop_order` parameter controls how parameters are iterated:

- **First element** = Outer loop (changes least frequently)
- **Last element** = Inner loop (changes most frequently)

**Example**: `['power', 'f_mod', 'f_dev']`
```
For power in [-25, -20, -15]:
    For f_mod in [16kHz, 20kHz, 25kHz]:
        For f_dev in [500kHz, 550kHz, 600kHz]:
            measure()
```

**Recommendation**: Put `'power'` first for thermal stability (minimizes number of power changes).

## Public Methods

### Configuration

#### `configure_sweep(sweep_params, odmr_params, stream_params)`

Configure the parameter sweep before starting.

**Parameters**:
- `sweep_params` (dict): Parameter arrays
  ```python
  {
      'power': np.array([-25, -20, -15]),  # dBm
      'f_mod': np.array([16e3, 20e3, 25e3]),  # Hz
      'f_dev': np.array([500, 550, 600])  # kHz
  }
  ```

- `odmr_params` (dict): ODMR scan configuration
  ```python
  {
      'frequency_start': 2.86e9,  # Hz
      'frequency_stop': 2.88e9,  # Hz
      'frequency_points': 1001,
      'run_time': 60,  # seconds
      'data_rate': 1000,  # Hz
      'multi_freq_mode': 'triple'
  }
  ```

- `stream_params` (dict): Time series recording configuration
  ```python
  {
      'n_time_traces': 32,  # Number of traces to record
      'trace_duration': 1.0,  # Seconds per trace
      'data_rate': 1000  # Hz
  }
  ```

### Control

#### `start_sweep()`
Start the parameter sweep from the beginning. Module must be configured first.

#### `pause_sweep()`
Pause the running sweep after the current measurement completes. State is saved for resumption.

#### `resume_sweep()`
Resume a paused sweep from where it left off.

#### `cancel_sweep()`
Cancel the running or paused sweep. Partial results are saved.

### Status

#### `get_sweep_status() → dict`
Get current sweep status.

**Returns**:
```python
{
    'state': 'idle'|'running'|'paused'|'cancelled',
    'current_index': int,
    'total_points': int,
    'best_sensitivity': float,  # nT/√Hz
    'best_parameters': dict,
    'current_folder': str
}
```

#### `get_current_results() → pd.DataFrame`
Get all completed results as pandas DataFrame.

**Columns**:
- `power_dbm`
- `f_mod_hz`
- `f_dev_khz`
- `linewidth_hz`
- `zc_slope_V_per_Hz`
- `sensitivity_nT_rtHz`
- `sensitivity_off_resonant_nT_rtHz` (if enabled)

## Signals

### Control Signals

| Signal | Parameters | Description |
|--------|------------|-------------|
| `sigSweepStarted` | `int total_points` | Emitted when sweep starts |
| `sigSweepPaused` | None | Emitted when sweep pauses |
| `sigSweepResumed` | None | Emitted when sweep resumes |
| `sigSweepFinished` | `str results_folder` | Emitted when sweep completes |
| `sigSweepCancelled` | None | Emitted when sweep is cancelled |

### Progress Signals

| Signal | Parameters | Description |
|--------|------------|-------------|
| `sigPointStarted` | `int index, dict params` | Before each measurement |
| `sigPointCompleted` | `int index, dict result` | After each measurement |
| `sigSweepProgress` | `dict progress` | Periodic progress updates |
| `sigError` | `str error_message` | Error occurred |

**`sigSweepProgress` dict**:
```python
{
    'current_idx': int,
    'total': int,
    'best_sens': float,
    'best_params': dict,
    'completion_percent': float
}
```

### Data Signals (for GUI plotting)

| Signal | Parameters | Description |
|--------|------------|-------------|
| `sigOdmrDataReady` | `ndarray freq, ndarray signal` | ODMR scan data |
| `sigFitDataReady` | `dict fit_result` | Resonance fit results |
| `sigASDDataReady` | `ndarray freq, ndarray asd` | ASD data |
| `sigTimeTraceReady` | `ndarray times, ndarray b_field` | Time trace data |

## Usage Examples

### Interactive (Jupyter Notebook)

```python
# Access module
sens_logic = qudi.module_instances['sensitivity_sweep_logic']

# Configure sweep
sweep_params = {
    'power': np.linspace(-25, -15, 10),
    'f_mod': np.array([16e3, 20e3, 25e3]),
    'f_dev': np.array([500, 550, 600])
}

odmr_params = {
    'frequency_start': 2.86e9,
    'frequency_stop': 2.88e9,
    'frequency_points': 1001,
    'run_time': 60,
    'data_rate': 1000,
    'multi_freq_mode': 'triple'
}

stream_params = {
    'n_time_traces': 32,
    'trace_duration': 1.0,
    'data_rate': 1000
}

sens_logic.configure_sweep(sweep_params, odmr_params, stream_params)

# Start sweep
sens_logic.start_sweep()

# Check status
status = sens_logic.get_sweep_status()
print(f"State: {status['state']}, Progress: {status['current_index']}/{status['total_points']}")

# Pause if needed
sens_logic.pause_sweep()

# Resume
sens_logic.resume_sweep()

# Get results
results_df = sens_logic.get_current_results()
print(results_df)
```

### GUI Integration

See `SensitivitySweepGui` for full integration example. Key pattern:

```python
# In GUI __init__
_sensitivity_logic = Connector(interface='SensitivitySweepLogic')

# In on_activate()
logic = self._sensitivity_logic()

# Connect signals (thread-safe with QueuedConnection)
logic.sigSweepProgress.connect(
    self._update_progress_bar,
    QtCore.Qt.QueuedConnection
)

logic.sigASDDataReady.connect(
    self._update_asd_plot,
    QtCore.Qt.QueuedConnection
)

# Emit start command
self.sigStartSweep.emit(sweep_params, odmr_params, stream_params)
logic.start_sweep()
```

## Data Output

### File Structure

```
<data_dir>/SensitivitySweep/YYYY-MM-DD_HHMMSS/
├── parameter_sweep_summary.csv  # All results in table
├── sweep_metadata.json           # Sweep configuration and metadata
├── P_-25.00dBm_fmod_16.0k_fdev_500.0k_20240120_153045_ODMR.dat
├── P_-25.00dBm_fmod_16.0k_fdev_500.0k_20240120_153045_ODMR.png
├── P_-25.00dBm_fmod_16.0k_fdev_500.0k_20240120_153045_fit.png
├── P_-25.00dBm_fmod_16.0k_fdev_500.0k_20240120_153045_ON-resonant_asd.dat
├── P_-25.00dBm_fmod_16.0k_fdev_500.0k_20240120_153045_ON-resonant_asd.png
├── P_-25.00dBm_fmod_16.0k_fdev_500.0k_20240120_153045_ON-resonant_timetraces.png
├── ...  (one set per measurement point)
```

### Summary CSV Format

Tab-separated with columns:
- `power_dbm`
- `f_mod_hz`
- `f_dev_khz`
- `linewidth_hz`
- `zc_slope_V_per_Hz`
- `sensitivity_nT_rtHz`
- `sensitivity_off_resonant_nT_rtHz` (if enabled)

### Metadata JSON

```json
{
    "total_measurements": 90,
    "total_planned": 90,
    "best_sensitivity_nT_rtHz": 12.3,
    "best_parameters": {
        "power": -20.0,
        "f_mod": 20000.0,
        "f_dev": 550.0
    },
    "sweep_loop_order": ["power", "f_mod", "f_dev"],
    "thermal_stabilization_time_s": 180,
    "which_zero_crossing": 2,
    "odmr_parameters": {...},
    "stream_parameters": {...},
    "timestamp": "2024-01-20T15:30:45"
}
```

## Performance Characteristics

### vs. RPyC Remote Control Approach

| Metric | RPyC (Old) | Qudi Module (New) | Improvement |
|--------|-----------|-------------------|-------------|
| Time series read latency | 50ms | <1ms | 50x faster |
| ODMR data transfer | 200ms | 5ms | 40x faster |
| Overhead per point | ~10s | ~1s | 10x faster |
| **75-point sweep overhead** | **12.5 minutes** | **1.25 minutes** | **~11 minutes saved** |

### Typical Sweep Duration

For a 75-point sweep (5 power × 5 f_mod × 3 f_dev):

- **ODMR scans**: 75 × 60s = 75 minutes
- **Thermal waits**: 4 × 180s = 12 minutes (power changes)
- **Time series**: 75 × (32 traces × 1s) = 40 minutes
- **Analysis/overhead**: ~5 minutes
- **Total**: ~132 minutes (2.2 hours)

Compare to RPyC approach: ~143 minutes (11 minutes longer due to serialization overhead)

## Troubleshooting

### Sweep Not Starting

**Symptoms**: `start_sweep()` raises RuntimeError

**Causes**:
- Sweep not configured: Call `configure_sweep()` first
- Module still running: Check `module_state()` is 'idle'
- Invalid parameters: Check arrays are non-empty and valid

### Fit Failures

**Symptoms**: Many points show `sensitivity_nT_rtHz: NaN`

**Solutions**:
- Adjust `min_fit_amplitude` and `min_feature_height` config options
- Check ODMR signal quality (sufficient contrast, good SNR)
- Increase `odmr_run_time` for better statistics
- Verify `n_most_prominent_peaks` is appropriate for your spectrum

### Time Series Data Issues

**Symptoms**: Sensitivity values are unrealistic or highly variable

**Solutions**:
- Check Red Pitaya streaming is working (`redpitaya_data_instream` activated)
- Verify `time_series_reader_logic` is properly connected
- Increase `n_time_traces` for better statistics
- Check lock-in demodulation settings (bandwidth, phase)

### Thermal Drift

**Symptoms**: Results drift systematically during sweep

**Solutions**:
- Increase `thermal_stabilization_time` (default 180s)
- Put 'power' first in `sweep_loop_order` to minimize power changes
- Allow full thermal equilibration before starting sweep
- Check laser stability

## Related Modules

- **ODMR Logic** (`odmr_logic.OdmrLogic`): Provides ODMR scanning and CW control
- **Time Series Reader Logic** (`time_series_reader_logic.TimeSeriesReaderLogic`): Provides streaming data acquisition
- **Red Pitaya DataInStream** (`redpitaya_data_instream.RedPitayaDataInStream`): Hardware interface for lock-in demodulation
- **Sensitivity Sweep GUI** (`sensitivity_sweep_gui.SensitivitySweepGui`): User interface for this module

## Migration from Old Scripts

### Old Approach (RPyC)
```python
# External script using RPyC
odmr_remote = OdmrRemoteControl()
odmr_remote.configure_mw_source(power, f_mod, f_dev)
freq, signal = odmr_remote.take_single_odmr_scan(...)
# Heavy serialization overhead for every call
```

### New Approach (Qudi Module)
```python
# Inside Qudi framework - zero overhead
odmr = self._odmr_logic()  # Direct connector access
odmr.set_scan_power(power)
mw = odmr._microwave()
mw.set_fm_parameters(enable=True, deviation_khz=f_dev, modulation_frequency=f_mod)
# No serialization - direct memory access
```

### Benefits

1. **Performance**: 10-50x faster (no serialization)
2. **Integration**: Native GUI support with real-time updates
3. **Reliability**: Thread-safe by design (Qt signals)
4. **Persistence**: Automatic state saving (StatusVar)
5. **Maintainability**: Standard Qudi patterns and logging

## References

- **Qudi Core Documentation**: https://ulm-iqo.github.io/qudi-core/
- **ODMR Logic**: `qudi-iqo-modules/src/qudi/logic/odmr_logic.py`
- **Time Series Reader**: `qudi-iqo-modules/src/qudi/logic/time_series_reader_logic.py`
- **DataInStream Interface**: `qudi-iqo-modules/src/qudi/interface/data_instream_interface.py`
