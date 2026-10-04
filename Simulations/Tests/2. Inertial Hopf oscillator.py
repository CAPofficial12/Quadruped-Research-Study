import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from collections import deque

dt = 0.005
mu = 1
frequency = 1
omega = 2 * math.pi * frequency

fps = 60
substep = int(round((1 / fps) / dt))
intervl = int(1000 / fps)

x, y = 2, 0
Vx, Vy = -1, 0
x_data = deque([x], maxlen=3000)
y_data = deque([y], maxlen=3000)
mass = 0.5
damping = 2

fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect("equal")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Inertial Hopf Oscillator")
point, = ax.plot([], [], "ro", markersize=8)
trail, = ax.plot([], [], "b-", alpha=0.5)

def update(frame):
    global x, y, Vx, Vy

    for _ in range(substep):
        r2 = x**2 + y**2

        tVx = (mu - r2) * x - omega * y
        tVy = (mu - r2) * y + omega * x

        Ax = damping * (tVx - Vx) / mass
        Ay = damping * (tVy - Vy) / mass

        Vx += Ax * dt
        Vy += Ay * dt

        x += Vx * dt
        y += Vy * dt

        x_data.append(x)
        y_data.append(y)

    point.set_data([x], [y])
    trail.set_data(x_data, y_data)
    return point, trail

ani = animation.FuncAnimation(fig, update, frames=1000, interval=intervl, blit=True, repeat=True)
plt.show()