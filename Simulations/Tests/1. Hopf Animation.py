import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

dt = 0.005
mu = 1
frequency = 1
omega = 2*math.pi*frequency

x, y = 5,5
Vx, Vy = 0,0
x_data, y_data = [x], [y]

fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-6, 6)
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Real-Time Velocity affecting Position")
point, = ax.plot([],[], "ro", markersize = 8)
trail, = ax.plot([],[], "b-", alpha=0.5)

def update(frame):
    global x, y, Vx, Vy
        
    r = math.sqrt(x**2 + y**2)
    Vx = (mu - r**2)*x - omega*y
    Vy = (mu - r**2)*y + omega*x
    x += Vx * dt
    y += Vy * dt

    x_data.append(x)
    y_data.append(y)

    point.set_data([x], [y])
    trail.set_data(x_data, y_data)

    return point, trail
ani = animation.FuncAnimation(fig, update, frames=1000, interval=dt*1000, blit=True, repeat=True)

plt.show()