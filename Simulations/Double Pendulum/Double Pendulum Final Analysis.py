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

print(df.shape)
print(df.head())
print(df.info())