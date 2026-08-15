import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from scipy import stats
import statsmodels.api as sm
import statsmodels.formula.api as smf

from pathlib import Path

script_dir = Path(__file__).resolve().parent
csv_fullpath = script_dir /"Summary"/f"Double Pendulum Mass Summary Statistics.csv"
df = pd.read_csv(csv_fullpath)
IV = "weight_mass"

primary_dvs = ["total_energy_relative_loss",
                  "trajectory_length",
                  "q1_dominant_frequency",
                  "q2_dominant_frequency",
                  "q1_decay_rate",
                  "q2_decay_rate",
                  "q1_velocity_max_abs",
                  "q2_velocity_max_abs",
                  "q1_acceleration_max_abs",
                  "q2_acceleration_max_abs"]
df_valid = df[df["valid"] == 1].copy