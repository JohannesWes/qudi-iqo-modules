# ODMR Frequency Tracking Module

Extended ODMR module with automatic frequency lock and tracking capabilities for tracking drifting NV center resonances.

## Overview

The ODMR Frequency Tracking module **inherits all functionality** from the standard ODMR module and adds hardware-based frequency locking features using PyRPL's FPGA-based lock. This allows automatic tracking of resonance frequency drift in real-time.

## Architecture

```
OdmrFrequencyTrackingLogic (inherits from OdmrLogic)
    └─ Inherits: ODMR scanning, CW control, data storage, fitting framework
    └─ Adds: Linear fit extraction, lock control, error signal streaming

OdmrTrackingGui (inherits from OdmrGui)
    └─ Inherits: All ODMR controls (scan, CW, menu, toolbar, fit dock)
    └─ Adds: Fit Controls Dock, Lock Controls Dock, Time Series Dock
```

## Files

- **`odmr_tracking_gui.py`**: Main GUI module, extends standard ODMR GUI with tracking features
- **`README.md`**: This documentation

The logic module is located at:
- **`qudi-iqo-modules/src/qudi/logic/odmr_frequency_tracking_logic.py`**

The hardware interface and implementation:
- **`qudi-iqo-modules/src/qudi/interface/odmr_freq_lock_interface.py`** - Abstract interface
- **`qudi-iqo-modules/src/qudi/hardware/redpitaya/redpitaya_odmr_lock.py`** - PyRPL wrapper

## Features

### Inherited from ODMR (via OdmrGui)
- ✅ All standard ODMR scanning functionality
- ✅ CW microwave control
- ✅ Multiple frequency range scanning
- ✅ Data fitting with various models
- ✅ Data saving and export
- ✅ Real-time plotting
- ✅ Menu bar (File, View, Settings)
- ✅ Toolbar with start/stop/save controls
- ✅ Status bar with elapsed time/scans

### New Frequency Tracking Features
- **Linear Fit Extraction**: Fit linear slope around resonance for lock configuration
- **Interactive Fit Region**: Draggable region selector on ODMR plot
- **Hardware-Based Locking**: PyRPL FPGA integral and PI control modes
- **Lock Configuration**: Easy setup of bandwidth, zero placement, and control mode
- **Independent Stream/Lock Control**: Error streaming and frequency lock operate independently
- **Stream Mode Selection**: Switch between error signal (LSB) and frequency correction (Hz) streaming
- **Real-Time Monitoring**:
  - Error signal or frequency correction time series plot (~30.5 kHz update rate)
  - Lock status indicators (locked, saturated, correction)
  - Selectable stream content (error vs correction)
- **Dock Widget Interface**:
  - Fit Controls Dock (fit region, slope results)
  - Lock Controls Dock (mode, parameters, stream/lock enable/disable)
  - Time Series Dock (error or correction plots with mode selection)

## Usage

### 1. Configuration

Add to your qudi configuration file (see example in config folder):

```yaml
logic:
    odmr_frequency_tracking_logic:
        module.Class: 'odmr_frequency_tracking_logic.OdmrFrequencyTrackingLogic'
        options:
            default_scan_mode: 'EQUIDISTANT_SWEEP'
            oversampling_factor: 5
            default_lock_bandwidth: 300  # Hz
            error_buffer_size: 10000
            status_poll_interval: 0.1    # seconds
        connect:
            microwave: 'mw_source_synthnv'        # Inherited from OdmrLogic
            data_scanner: 'redpitaya_scanner'     # Inherited from OdmrLogic
            odmr_lock_hw: 'redpitaya_odmr_lock'   # New: lock hardware
            error_streamer: 'redpitaya_stream'    # New: error streaming

hardware:
    redpitaya_odmr_lock:
        module.Class: 'redpitaya.redpitaya_odmr_lock.RedPitayaOdmrLockHardware'
        options:
            redpitaya_config_name: 'rpy_shared_config'
            redpitaya_hostname: '10.203.129.28'

gui:
    odmr_tracking_gui:
        module.Class: 'odmr_tracking.odmr_tracking_gui.OdmrTrackingGui'
        connect:
            odmr_logic: 'odmr_frequency_tracking_logic'
```

### 2. Workflow

1. **Start qudi** and activate the ODMR Tracking GUI
2. **Configure scan** using standard ODMR controls (inherited)
3. **Start scan** using toolbar button or menu
4. **Identify resonance** visually on the ODMR plot
5. **Select fit region**:
   - Drag the shaded region on the plot, OR
   - Enter frequency min/max in Fit Controls Dock
6. **Fit resonance**:
   - Click "Fit Resonance" button
   - View slope (LSB/Hz) and R² in dock
   - Fit curve appears as dashed line on plot
7. **Configure lock**:
   - Select mode: Integral (simple) or PI (faster)
   - Set bandwidth (typical: 100-1000 Hz)
   - Set Zero Ratio α if using PI mode (typical: 3.0)
   - Click "Configure" button
8. **(Optional) Start streaming without lock**:
   - Select stream mode: Error Signal (LSB) or Frequency Correction (Hz)
   - Click "Start Stream" to monitor signals without enabling lock
   - Useful for verification and diagnostics
9. **Enable lock**:
   - Click "Enable Lock" button (automatically starts streaming if not running)
   - Monitor status indicators:
     - **Locked**: Green = acquired, Red = not locked
     - **Saturated**: Shows if correction hit limits
     - **Error [LSB]**: Current demodulated error signal
     - **Correction [Hz]**: Frequency offset being applied
10. **Monitor tracking**:
    - Time Series plot shows error signal or frequency correction
    - Adjust bandwidth if needed for stability
11. **Disable lock**:
    - Click "Disable Lock" when done (streaming continues)
    - Click "Stop Stream" to stop data acquisition
    - Use "Clear" button to reset integrator if saturated

### 3. Lock Tuning Tips

**Integral Mode** (recommended for starting):
- Start with BW = 300 Hz
- Stable, no overshoot, good noise rejection
- Increase BW for faster tracking (max ~1 kHz)
- Decrease if lock oscillates

**PI Mode** (advanced):
- Start with BW = 300 Hz, α = 3.0 (balanced zero placement)
- Faster response than integral-only
- Zero frequency = BW / α (e.g., 300 Hz / 3.0 = 100 Hz zero)
- **α = 2.0**: Aggressive (zero at BW/2, faster but may overshoot)
- **α = 3.0**: Balanced (zero at BW/3, recommended)
- **α = 4.0**: Conservative (zero at BW/4, slower but very stable)
- Use for rapid frequency changes

**Stream Mode Selection**:
- **Error Signal (LSB)**: Raw demodulated error signal, useful for diagnostics
- **Frequency Correction (Hz)**: FTW correction applied to microwave, shows actual frequency drift

### 4. Independent Stream/Lock Operation

The module supports **independent control** of streaming and locking:

**Use Cases**:
1. **Stream-only**: Monitor error signal without enabling lock (diagnostics, verification)
2. **Lock-only**: Enable lock (streaming starts automatically for lock operation)
3. **Both**: Stream and lock operate together (normal tracking operation)

**Control Flow**:
- Stream can start/stop without affecting lock state
- Lock automatically starts streaming if not running (required for operation)
- Stopping lock does NOT stop streaming (allows continued monitoring)

## Implementation Details

### Logic Module (`odmr_frequency_tracking_logic.py`)

Key methods:

#### `fit_resonance(freq_min, freq_max)`
Performs linear fit on ODMR scan data within specified range.
- Uses parent's `signal_data` and `frequency_data`
- Returns dict with slope, R², fit curve
- Emits `sigFitCompleted` signal to GUI

#### `configure_lock(bandwidth_hz, slope_lsb_per_hz)`
Configures integral-only lock mode.
- Calls hardware interface `set_bandwidth()`
- PyRPL calculates μ (integral gain) and programs FPGA

#### `configure_lock_pi(bandwidth_hz, slope_lsb_per_hz, zero_ratio)`
Configures PI lock mode with zero placement.
- Calls hardware interface `set_bandwidth_pi()`
- PyRPL calculates μ and Kp (proportional gain), programs FPGA
- `zero_ratio` (α): Controls zero placement, range [2.0, 4.0], default 3.0
  - Zero frequency = bandwidth_hz / zero_ratio
  - Higher α = more conservative (lower zero frequency)

#### `set_stream_mode(mode)`
Set streaming mode: 'error' or 'correction'.
- Cannot change while streaming is active
- Controls what signal is streamed to Time Series plot

#### `start_error_stream()` / `stop_error_stream()`
Start/stop error signal streaming independently of lock.
- Allows monitoring without enabling lock
- Lock automatically starts streaming if needed

#### `start_tracking()` / `stop_tracking()`
Enable/disable frequency lock.
- Automatically starts streaming if not active (required for lock)
- Stopping lock does NOT stop streaming
- Starts/stops status polling QTimer

#### `clear_integrator()`
Reset lock integrator to zero.
- Useful when lock is saturated
- Does not disable lock
- Calls hardware interface `clear()`

### GUI Module (`odmr_tracking_gui.py`)

**Inherited from OdmrGui:**
- All scan control widgets
- CW control widget
- Fit dock widget (hidden in tracking mode)
- Menu bar and toolbar
- Settings dialog
- Plot widget

**Added in OdmrTrackingGui:**
- Fit region overlay (LinearRegionItem)
- Fit curve overlay (PlotDataItem)
- Fit Controls Dock (fit range, button, results)
- Lock Controls Dock (mode, parameters, buttons, status)
- Time Series Dock (error/correction plot with mode selection)
- Stream mode radio buttons (Error Signal / Frequency Correction)
- Independent Start/Stop Stream buttons
- Independent Enable/Disable Lock buttons

**Key override:**
- `restore_default_view()`: Adds tracking docks to window layout
- `_hide_unused_parent_elements()`: Hides ODMR fit dock and matrix plot

### Hardware Interface

**OdmrFreqLockInterface** (abstract):
Defines required methods:
- `set_bandwidth(bandwidth_hz, slope_lsb_per_hz)` - Integral mode
- `set_bandwidth_pi(bandwidth_hz, slope_lsb_per_hz, zero_ratio)` - PI mode
  - `zero_ratio`: Range [2.0, 4.0], controls zero placement
- `enable_lock(enable)` - Turn on/off
- `get_status()` - Returns dict with enabled, locked, saturated, error_lsb, correction_hz
- `clear()` - Reset integrator
- `get_constraints()` - Returns hardware limits

**RedPitayaOdmrLockHardware** (implementation):
- Wraps PyRPL `odmrfreqlock` module
- Manages PyRPL instance via resource manager
- Thread-safe (runs on main thread)
- Constraints:
  - Bandwidth range: 10-10000 Hz
  - Slope range: 1e-6 to 1e6 LSB/Hz
  - Zero ratio range: 2.0-4.0
  - Max correction: 62.5 MHz (Nyquist limit)

## Dock Widget Layout

Default layout after `restore_default_view()`:

```
┌────────────────────────────────────────────────────────┐
│ Menu Bar: File | View | Settings                       │
├────────────────────────────────────────────────────────┤
│ Toolbar: [Start] [Stop] [Resume] [Save] [CW]          │
├────────────────────────────────────────────────────────┤
│  ┌──────────────────────┐  ┌─────────────────────┐    │
│  │                      │  │ Fit Controls        │    │
│  │   ODMR Plot          │  │ Freq Min:           │    │
│  │   (with fit region)  │  │ Freq Max:           │    │
│  │                      │  │ [Fit Resonance]     │    │
│  │                      │  │ Slope: --           │    │
│  │                      │  │ R²: --              │    │
│  │                      │  ├─────────────────────┤    │
│  │                      │  │ Lock Controls       │    │
│  └──────────────────────┘  │ ○ Integral ○ PI    │    │
│                            │ BW: 300 Hz          │    │
│                            │ Zero Ratio (α): 3.0 │    │
│                            │ [Configure] [Clear] │    │
│                            │ Stream (independent):    │
│                            │ [Start] [Stop]      │    │
│                            │ Lock:               │    │
│                            │ [Enable] [Disable]  │    │
│                            │ Locked: OFF         │    │
│                            │ Saturated: --       │    │
│                            │ Error [LSB]: --     │    │
│                            │ Correction [Hz]: -- │    │
│                            └─────────────────────┘    │
├────────────────────────────────────────────────────────┤
│  Time Series Dock                                      │
│  Stream Mode: ○ Error Signal (LSB) ○ Freq Corr (Hz)   │
│  [Plot showing selected signal vs time]                │
└────────────────────────────────────────────────────────┘
│ Status Bar: Elapsed Time: 0:00:00 | Scans: 0          │
└────────────────────────────────────────────────────────┘
```

## Troubleshooting

**GUI won't activate**:
- Check connector name is `odmr_logic` (standard OdmrGui connector)
- Verify tracking logic implements OdmrLogic interface (via inheritance)

**Fit button does nothing**:
- Ensure ODMR scan has completed at least once
- Check fit range is within scan range
- Verify fit range has at least 2 data points

**Lock won't enable**:
- Run fit first to extract slope
- Click "Configure" before "Enable Lock"
- Check PyRPL connection and odmrfreqlock module availability
- Verify IQ demodulator is configured in PyRPL

**Lock won't acquire**:
- Check error signal polarity (may need to adjust in PyRPL)
- Verify fit range selection (should be on slope, not at dip center)
- Try increasing bandwidth
- Check that demodulation frequency matches FM modulation

**Lock saturates immediately**:
- Click "Clear" to reset integrator
- Check fit range and slope are correct
- Reduce bandwidth
- Verify error signal is near zero before enabling

**Error signal not updating**:
- Click "Start Stream" to begin data acquisition
- Check error_streamer is configured correctly
- Verify scan module is in streaming mode in PyRPL
- Check that IQ demodulator output is routed to scan module

**Cannot change stream mode**:
- Stream mode can only be changed when streaming is stopped
- Click "Stop Stream" first, then change mode
- Mode selection radio buttons are disabled while streaming

## Development Status

**Current Status**: ✅ **COMPLETE AND OPERATIONAL**

**Implemented**:
- [x] Inheritance-based architecture (Logic extends OdmrLogic, GUI extends OdmrGui)
- [x] Interactive fit region selection on plot
- [x] Linear fitting with slope extraction
- [x] Integral and PI lock mode configuration with zero placement
- [x] Independent stream and lock control
- [x] Stream mode selection (error signal vs frequency correction)
- [x] Real-time error signal plotting
- [x] Real-time frequency correction plotting
- [x] Lock status monitoring
- [x] Hardware abstraction via interface
- [x] PyRPL integration via resource manager
- [x] StatusVar persistence for user settings
- [x] Complete dock widget layout

**Future Enhancements**:
- [ ] Auto-calibration (polarity detection, slope measurement)
- [ ] Data export (tracking history, error time series)
- [ ] Advanced diagnostics (PSD, Allan variance)
- [ ] Unit tests

## License

Copyright (c) 2021, the qudi developers.

LGPL v3 - See main qudi license file.

## Contact

For questions or contributions, refer to the main qudi-core repository.
