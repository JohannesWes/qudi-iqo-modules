import matplotlib
from qm import QuantumMachinesManager
from qm.qua import *
from qm import SimulationConfig
import numpy as np
import matplotlib.pyplot as plt
from configuration_FM import *
matplotlib.use("Qt5Agg")

simulate = False

# DEFINITION OF THE CHIRP RATES
# -----------------------------------------------------------------------------------------

# number of time-segments for the FM / chirp: frequency is changed after each segment
n_segments = 100
dt = FM_period_duration / n_segments
omega_FM = 2 * np.pi / FM_period_duration

time_vec = dt * np.array(range(n_segments + 1))
freq_vec = f_dev * np.sin(omega_FM * time_vec)

# rates with that the frequency is changed after each segment
rates = (np.diff(freq_vec) / (FM_period_duration / n_segments)).astype(int).tolist()
units = "Hz/nsec"


# EXECUTION OF THE OPX PROGRAM
# -----------------------------------------------------------------------------------------
qmm = QuantumMachinesManager(opx_ip_adress)
qm = qmm.open_qm(config)

# on the IQ-ports a sine with varying frequency is generated (chirp)
# on the LIA port a sine with constant frequency - the FM frequency - is generated (reference signal)
# the method with the zero pulses in the infinite_loop and the sticky parameters (in configuration_FM.py) was suggested
# by the QM team (see their discord channel)
with program() as prog:
    play("const", "ensemble", chirp=(rates, units))
    play("const_single", "LIA")
    with infinite_loop_():
        play("zero", "ensemble", chirp=(rates, units))
        play("zero_single", "LIA")

if simulate:
    print("Make sure that the simulation time is long enough to see the whole chirp, but also not too long.")
    job = qmm.simulate(config, prog, SimulationConfig(int(SIM_TIME // 4)))  # in clock cycles, 4 ns per clock cycle
    samples = job.get_simulated_samples()

else:
    qm = qmm.open_qm(config)
    my_job = qm.execute(prog)


# PLOTTING SIMULATION RESULTS
# -----------------------------------------------------------------------------------------
if simulate:
    analog_1 = samples.con1.analog["1"]

    analog_2 = samples.con1.analog["3"]

    times = SIM_TIME / len(analog_2) * np.arange(len(analog_2)) / 1e9
    indices = np.arange(len(analog_2))

    fig, ax = plt.subplots(2, 1)

    ax[0].plot(times, analog_2)
    ax[0].set_xlabel("t [s]")
    ax[0].set_ylabel("U [V]")
    ax[0].set_title("Referenzsignal -> Lock-In Amplifier")
    ax[0].set_xlim(np.min(times), np.max(times))

    NFFT = 2 ** 11
    Fs = 1e9
    Pxx, freqs, bins, im = ax[1].specgram(analog_1, NFFT=NFFT, Fs=Fs, noverlap=1000, cmap=plt.cm.gist_heat)

    # ax1.set_xticklabels((ax1.get_xticks() * 1e6).astype(int))
    # ax1.set_yticklabels((ax1.get_yticks() / 1e6).astype(int))
    # plt.title("Sinusoidal Chirp")
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("f [Hz]")
    ax[1].set_title("Spectogram FM Signal -> IQ-Mixer -> MW Antenne")

    plt.show()
