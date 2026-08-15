import csv
from pathlib import Path
import numpy as np
import pandas as pd
from scipy import signal, stats


def CSV_read(test_num):
    script_dir = Path(__file__).resolve().parent
    csv_fullpath = script_dir / "Full Results" / f"Double Pendulum. Test {test_num}.csv"
    return pd.read_csv(csv_fullpath)


def intro(test_num):
    df = CSV_read(test_num)
    q1_initial = df["q1"].iat[0]
    q2_initial = df["q2"].iat[0]
    sim_time = df["time"].iat[-1] - df["time"].iat[0]
    time_step = df["time"].diff().mean()
    valid = (sim_time/time_step)+1 ==  len(df)
    initial_state = [test_num, q1_initial, q2_initial, sim_time, time_step, valid]
    return initial_state

def Motion_Stats(values):
    mean = np.mean(values)
    median = np.median(values)
    std = np.std(values)

    LQ, UQ = np.quantile(values, [0.25, 0.75], method="linear")
    IQR = UQ - LQ

    greatest = np.max(values)
    stats = [mean, std, median, IQR, greatest]
    return stats

def Energy(energy):
    initial =energy.iat[0]
    final = energy.iat[0]
    std = np.std(energy)
    mean = np.mean(energy)
    least = np.min(energy)
    greatest = np.max(energy)
    energy = [initial, final, std, mean, least, greatest]
    return energy

def dom_frep(joint, time):
    angles = signal.detrend(joint)
    n = len(angles)
    dt = time.diff().median()
    frequencies = np.fft.rfftfreq(n, d=dt)
    fft_val = np.fft.rfft(angles)
    power = np.abs(fft_val)**2
    power[0] = 0
    dominant_index = np.argmax(power)
    dominant_frequency = frequencies[dominant_index]
    return dominant_frequency

def decay(joint, time):
    peaks, _ = signal.find_peaks(joint, prominence=0.01)

    peak_times = time[peaks]
    peak_values = joint[peaks]

    # Only positive amplitudes can be log-transformed
    valid = peak_values > 0

    peak_times = peak_times[valid]
    peak_values = peak_values[valid]

    log_amplitude = np.log(peak_values)

    slope, intercept, r_value, p_value, std_err = stats.linregress(
        peak_times,
        log_amplitude
    )

    decay_rate = -slope
    return decay_rate

def oscillations(test_num, joint, time):
    dom_q1 = dom_frep(joint, time)
    peaks, prop = signal.find_peaks(joint, prominence = 0.01)
    oscil_count = len(peaks)
    rate = decay(joint, time)
    spring = [dom_q1, oscil_count, rate]
    return spring

def summary(test_num):
    df = CSV_read(test_num)

    #Initial Condition
    initial = intro(test_num)

    #Weight Distance for original position
    displacement = np.sqrt((df["weight_x"] - df["weight_x"].iat[0])**2 + (df["weight_y"] - df["weight_y"].iat[0])**2 + (df["weight_z"] - df["weight_z"].iat[0])**2)
    weight = Motion_Stats(displacement)
    step_distance = np.sqrt(
        df["weight_x"].diff()**2 +
        df["weight_y"].diff()**2 +
        df["weight_z"].diff()**2
    )
    trajectory_length = step_distance.sum()
    weight += displacement[0] + trajectory_length

    #Weight Speed
    speed = np.sqrt((df["weight_vx"])**2 + (df["weight_vy"])**2 + (df["weight_vz"])**2)
    weight_velocity = Motion_Stats(speed)

    #Weight Acceleration
    acceleration = np.sqrt((df["weight_ax"])**2 + (df["weight_ay"])**2 + (df["weight_az"])**2)
    weight_acc = Motion_Stats(acceleration)

    #joint angles
    q1 = Motion_Stats(df["q1"])
    q2 = Motion_Stats(df["q2"])

    #Joint Velocities
    q1_v = Motion_Stats(df["q1_dot"])
    q2_v = Motion_Stats(df["q2_dot"])

    #Joint Accelerations
    q1_acc = Motion_Stats(df["q1_ddot"])
    q2_acc = Motion_Stats(df["q2_ddot"])

    #Energies
    Kinetic = Energy(df["kinetic energy"])
    Potential = Energy(df["potential energy"])
    Total = Energy(df["total energy"])

    #Spring Values
    Spring_q1 = oscillations(test_num, df["q1"], df["time"])
    Spring_q2 = oscillations(test_num, df["q2"], df["time"])

    #Final Position
    q1_final = df["q1"].iat[-1]
    q2_final = df["q2"].iat[-1]
    Vq1_final = df["q1_dot"].iat[-1]
    Vq2_final = df["q2_dot"].iat[-1]
    Aq1_final = df["q1_ddot"].iat[-1]
    Aq2_final = df["q2_ddot"].iat[-1]
    final_state = [q1_final, q2_final, Vq1_final, Vq2_final, Aq1_final, Aq2_final]

    result = []
    result =  initial+ weight + weight_velocity+ weight_acc+ q1+ q2+ q1_v+ q2_v+ q1_acc+ q2_acc+ Kinetic+ Potential+ Total+ Spring_q1+ Spring_q2+ final_state

def main():
    script_dir = Path(__file__).resolve().parent
    csv_fullpath = script_dir / f"Double Pendulum Summary Stats.csv"
    with open(csv_fullpath, "w", newline="\n") as file:
        writer = csv.writer(file)
        writer.writerow([
            "test_id",
            "q1_initial",
            "q2_initial",
            "simulation_duration",
            "timestep",
            "valid",

            "weight_displacement_mean",
            "weight_displacement_std",
            "weight_displacement_median",
            "weight_displacement_iqr",
            "weight_displacement_max",
            "weight_displacement_final",
            "trajectory_length",

            "speed_mean",
            "speed_std",
            "speed_median",
            "speed_iqr",
            "speed_max",

            "acceleration_mean",
            "acceleration_std",
            "acceleration_median",
            "acceleration_iqr",
            "acceleration_max",

            "q1_mean",
            "q1_std",
            "q1_median",
            "q1_iqr",
            "q1_max_abs",

            "q2_mean",
            "q2_std",
            "q2_median",
            "q2_iqr",
            "q2_max_abs",

            "q1_velocity_mean",
            "q1_velocity_std",
            "q1_velocity_median",
            "q1_velocity_iqr",
            "q1_velocity_max_abs",

            "q2_velocity_mean",
            "q2_velocity_std",
            "q2_velocity_median",
            "q2_velocity_iqr",
            "q2_velocity_max_abs",

            "q1_acceleration_mean",
            "q1_acceleration_std",
            "q1_acceleration_median",
            "q1_acceleration_iqr",
            "q1_acceleration_max_abs",

            "q2_acceleration_mean",
            "q2_acceleration_std",
            "q2_acceleration_median",
            "q2_acceleration_iqr",
            "q2_acceleration_max_abs",

            "initial_kinetic_energy",
            "final_kinetic_energy",
            "kinetic_energy_std",
            "kinetic_energy_mean",
            "kinetic_energy_min",
            "kinetic_energy_max",

            "initial_potential_energy",
            "final_potential_energy",
            "potential_energy_std",
            "potential_energy_mean",
            "potential_energy_min",
            "potential_energy_max",

            "initial_total_energy",
            "final_total_energy",
            "total_energy_std",
            "total_energy_mean",
            "total_energy_min",
            "total_energy_max",

            "q1_dominant_frequency",
            "q1_oscillation_count",
            "q1_decay_rate",

            "q2_dominant_frequency",
            "q2_oscillation_count",
            "q2_decay_rate",

            "final_q1",
            "final_q2",
            "final_q1_velocity",
            "final_q2_velocity",
            "final_q1_acceleration",
            "final_q2_acceleration"
        ])

        for i in range(1, 901):
            result = summary(i)
            writer.writerow(result)

result = summary(1)

print("Number of columns:", len(result))
print("Number of headers:", len(result.columns))
