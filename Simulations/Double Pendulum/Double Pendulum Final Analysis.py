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

print(df[IV].describe())
print(df[IV].value_counts().sort_index())

plt.hist(df[IV], bins=20)
plt.plot(df["test_id"], df[IV])
plt.xlabel("Mass")
plt.ylabel("Number of Simulations")
plt.show()