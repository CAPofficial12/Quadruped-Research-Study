
import matplotlib.pyplot as plt

dt = 0.005
fps = 60
substep = max(1, round((1 / fps) / dt))
interval = 1000 / fps

x1, x2 = -1.0, 5.0
y1, y2 = 0.0, 0.0
z1, z2 = max(0, x1), max(0, x2)

st = 1.0
a12, a21 = 2.0, 2.0
bt = 2.5
Taut_x, Taut_y = 0.5, 2.0


x3, x4 = 1.0, -5.0
y3, y4 = 0.0, 0.0
z3, z4 = max(0, x3), max(0, x4)

sl = 1.0
a34, a43 = 2.0, 2.0
bl = 2.5
Taul_x, Taul_y = 0.5, 2.0

x_data = []
y_data = []
Graph_SIZE = 1

def matsuoka(x1, x2, y1, y2, z1, z2,
             s, a12, a21, b, Tx, Ty):

    for _ in range(substep):
        x1_dot = (-x1 - b * y1 - a12 * z2 + s) / Tx
        y1_dot = (-y1 + z1) / Ty

        x2_dot = (-x2 - b * y2 - a21 * z1 + s) / Tx
        y2_dot = (-y2 + z2) / Ty

        x1 += x1_dot * dt
        x2 += x2_dot * dt
        y1 += y1_dot * dt
        y2 += y2_dot * dt

        z1 = max(0.0, x1)
        z2 = max(0.0, x2)

    return x1, x2, y1, y2, z1, z2

for i in range(5000):

    x1, x2, y1, y2, z1, z2 = matsuoka(
        x1, x2, y1, y2, z1, z2,
        st, a12, a21, bt, Taut_x, Taut_y
    )

    x3, x4, y3, y4, z3, z4 = matsuoka(
        x3, x4, y3, y4, z3, z4,
        sl, a34, a43, bl, Taul_x, Taul_y
    )

    output1 = z2 - z1
    output2 = z4 - z3

    x_data.append(output1)
    y_data.append(output2)

fig, ax = plt.subplots()

ax.set_xlim(-Graph_SIZE, Graph_SIZE)
ax.set_ylim(-Graph_SIZE, Graph_SIZE)
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("Knee Angle")
ax.set_ylabel("Hip Angle")
ax.set_title("Matsuoka Simulation")
ax.grid(True, alpha=0.3)

ax.plot(x_data, y_data, "r-", alpha=0.6, linewidth=1.5, label="Angle movement")

ax.legend()
plt.show()
