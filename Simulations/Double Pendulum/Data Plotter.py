import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

test_num = 2
IV = "Mass"
script_dir = Path(__file__).resolve().parent
csv_fullpath = script_dir / f"Summary" /f"Double Pendulum {IV} Summary Statistics.csv"
df = pd.read_csv(csv_fullpath)
all_variables = ["test_id",
            "gravity",
            "weight_mass",
            "q1_initial",
            "q2_initial",
            "q1_length",
            "q2_length",
            "q1_dampening",
            "q2_dampening",
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
            "total_energy_loss",
            "total_energy_relative_loss",

            "q1_dominant_frequency",
            "q1_oscillation_count",
            "q1_decay_rate",
            "q1_decay_r2",
            "q1_peak_amplitude_initial",
            "q1_peak_amplitude_final",
            "q1_peak_amplitude_mean",
            "q1_peak_amplitude_std",
            "q1_mean_period",
            "q1_period_std",

            "q2_dominant_frequency",
            "q2_oscillation_count",
            "q2_decay_rate",
            "q2_decay_r2",
            "q2_peak_amplitude_initial",
            "q2_peak_amplitude_final",
            "q2_peak_amplitude_mean",
            "q2_peak_amplitude_std",
            "q2_mean_period",
            "q2_period_std",

            "final_q1",
            "final_q2",
            "final_q1_velocity",
            "final_q2_velocity",
            "final_q1_acceleration",
            "final_q2_acceleration"]

main_variables = ["total_energy_relative_loss",
                  "trajectory_length",
                  "q1_dominant_frequency",
                  "q2_dominant_frequency",
                  "q1_decay_rate",
                  "q2_decay_rate",
                  "q1_velocity_max_abs",
                  "q2_velocity_max_abs",
                  "q1_acceleration_max_abs",
                  "q2_acceleration_max_abs"]

# Plot
x = "weight_mass"
for i in range(len(main_variables)):
    plt.figure(i)
    plt.plot(df[x], df[main_variables[i]])
    plt.title(x + " vs " + main_variables[i])

# Grid
plt.grid(True)

# Show graph
plt.show()