import numpy as np
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
K = 0.5

x_data = []
y_data = []

u1 = np.array([x1, x2])
u2 = np.array([x3, x4])

Graph_SIZE = 5


def matsuoka(x1, x2, y1, y2, z1, z2,
             s, a12, a21, b, Tx, Ty, K, phase_offset, u_own, u_other, neuron_index):    
    
    theta1 = np.arctan2(u_own[1], u_own[0])
    theta2 = np.arctan2(u_other[1], u_other[0])
    
    if neuron_index == 0:
        phases = np.array([theta1, theta2])
    else:
        phases = np.array([theta2, theta1])
        
    complex_phases = np.exp(1j * phases)
    Z = np.mean(complex_phases)
    R = np.abs(Z)
    psi = np.angle(Z)

    C = K * R * np.sin(psi - phases[0] - phase_offset)

    x1_dot = (-x1 - b * y1 - a12 * z2 + s + C) / Tx
    y1_dot = (-y1 + z1) / Ty

    x2_dot = (-x2 - b * y2 - a21 * z1 + s - C) / Tx
    y2_dot = (-y2 + z2) / Ty

    x1 += x1_dot * dt
    x2 += x2_dot * dt

    y1 += y1_dot * dt
    y2 += y2_dot * dt

    z1 = max(0.0, x1)
    z2 = max(0.0, x2)

    return x1, x2, y1, y2, z1, z2


for i in range(10000):

    x1, x2, y1, y2, z1, z2 = matsuoka(
        x1, x2, y1, y2, z1, z2,
        st, a12, a21, bt, Taut_x, Taut_y, K, 0.0, u1, u2, 0
    )

    x3, x4, y3, y4, z3, z4 = matsuoka(
        x3, x4, y3, y4, z3, z4,
        sl, a34, a43, bl, Taul_x, Taul_y, K, np.pi, u2, u1, 1
    )

    output1 = z2 - z1
    output2 = z4 - z3

    x_data.append(output1)
    y_data.append(output2)

    u1 = np.array([x1, x2])
    u2 = np.array([x3, x4])

fig, ax = plt.subplots()

ax.set_xlim(-Graph_SIZE, Graph_SIZE)
ax.set_ylim(-Graph_SIZE, Graph_SIZE)
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("Knee Angle (Oscillator 1)")
ax.set_ylabel("Hip Angle (Oscillator 2)")
ax.set_title("Matsuoka System via Kuramoto Mean-Field Coupling")
ax.grid(True, alpha=0.3)

ax.plot(x_data, y_data, "b-", alpha=0.7, linewidth=1.5, label="Steady Gait Orbit")

ax.legend()
plt.show()
