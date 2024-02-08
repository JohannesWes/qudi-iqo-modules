import numpy as np

##############
# Parameters #
##############
# ----------------------------------------------------------------------------------------------------------------------
voltage_opx = 0.5  # peak (not peak-to-peak) output voltage of the opx
# corrections for I & Q voltages; values from IQ-calibration script
I_offset, Q_offset = 0.00929, -0.00587
# corrections for g and phi; values from IQ-calibration script
g_cor, phi_cor = 0.03282, -0.18193

f_mod = 5.0e3  # modulation frequency
f_dev = 200.00e3  # deviation of the modulation
f_base = 200.00e6  # frequency around that we modulate: base frequency chosen as middle of bandwidth (400 MHz) of OPX
ensemble_lo = 2.73e9

opx_ip_adress = "192.168.88.10"

# ----------------------------------------------------------------------------------------------------------------------

# duration of one FM period (in ns); also serves as the pulse length; pulse length must be integer multiple of 4
FM_period_duration = int(1 / f_mod / 1e-9) - int(1 / f_mod / 1e-9) % 4
print(f"FM_period_duration [ns]: {FM_period_duration}")

SIM_TIME = int(10 * FM_period_duration)
print(f"SIM_TIME [ns]: {SIM_TIME}")


def IQ_imbalance(g, phi):
    c = np.cos(phi)
    s = np.sin(phi)
    n = 1 / ((1 - g ** 2) * (2 * c ** 2 - 1))
    return [float(n * x) for x in [(1 - g) * c, (1 + g) * s, (1 - g) * s, (1 + g) * c]]


# CONFIGURATION
# ----------------------------------------------------------------------------------------------------------------------

config = {
    "version": 1,
    "controllers": {
        "con1": {
            "type": "opx1",
            "analog_outputs": {
                # the voltage offsets correct for the LO Leakage; They are the outputs of the IQ-calibration script
                1: {"offset": I_offset},
                2: {"offset": Q_offset},
                3: {"offset": +0.0}
            },
        }
    },
    "elements": {
        "ensemble": {
            "mixInputs": {
                "I": ("con1", 1),
                "Q": ("con1", 2),
                "lo_frequency": ensemble_lo,
                "mixer": "mixer_ensemble"
            },
            "intermediate_frequency": f_base,
            'sticky': {
                'analog': True,
                'duration': 200
            },
            "operations": {
                "const": "constPulse",
                "zero": "zeroPulse",
            },
        },
        "LIA": {
            "singleInput": {"port": ("con1", 3)},
            "intermediate_frequency": f_mod,
            'sticky': {
                'analog': True,
                'duration': 200
            },
            "operations": {
                "const_single": "constPulse_single",
                "zero_single": "zeroPulse_single",
            },
        },
    },
    "pulses": {
        "constPulse": {
            "operation": "control",
            "length": FM_period_duration,  # in ns; multiple of 4 ns (clock cycle)
            "waveforms": {
                "I": "const_wf",
                "Q": "const_wf"
            },
        },
        "zeroPulse": {
            "operation": "control",
            "length": FM_period_duration,  # in ns; multiple of 4 ns (clock cycle)
            "waveforms": {
                "I": "zero_wf",
                "Q": "zero_wf"
            },
        },
        "constPulse_single": {
            "operation": "control",
            "length": FM_period_duration,  # in ns; multiple of 4 ns (clock cycle)
            "waveforms": {"single": "const_wf"},
        },
        "zeroPulse_single": {
            "operation": "control",
            "length": FM_period_duration,  # in ns; multiple of 4 ns (clock cycle)
            "waveforms": {"single": "zero_wf"},
        },
    },
    "waveforms": {
        "const_wf": {"type": "constant", "sample": voltage_opx},
        "zero_wf": {"type": "constant", "sample": 0.0},
    },
    "mixers": {
        "mixer_ensemble": [
            {
                "intermediate_frequency": f_base,
                "lo_frequency": ensemble_lo,
                "correction": IQ_imbalance(g_cor, phi_cor),
            },
        ],
    }
}
