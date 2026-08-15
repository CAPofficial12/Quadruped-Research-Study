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

control = [
    "gravity",
    "q1_initial",
    "q2_initial",
    "q1_length",
    "q2_length",
    "q1_dampening",
    "q2_dampening",
    "simulation_duration",
    "timestep"
]
for ctrl in control:
    print(np.max(np.diff(df[ctrl])))