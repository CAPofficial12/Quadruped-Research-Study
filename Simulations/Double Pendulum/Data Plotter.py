import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

test_num = 2
script_dir = Path(__file__).resolve().parent
csv_fullpath = script_dir / "Full Gravity Results" /f"Double Pendulum. Test {test_num}.csv"
df = pd.read_csv(csv_fullpath)

# Plot
x = "weight_z"
y = "weight_y"

plt.plot(df[x], df[y])

# Labels
plt.xlabel(x + " (m)")
plt.ylabel(y + " (m)")
plt.title(x + " vs " +  y)

# Grid
plt.grid(True)

# Show graph
plt.show()