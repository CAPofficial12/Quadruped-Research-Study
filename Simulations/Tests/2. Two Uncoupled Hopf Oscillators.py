import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

dt = 0.005
mu = 1
frequency = 1
omega = 2*math.pi*frequency

fps = 60
substep = int(round((1/fps)/dt))
intervl = int(1000/fps)

x1, y1 = -1,5
Vx1, Vy1 = 1, -10
x1_data, y1_data = [x1], [y1]

x2, y2 = 5,5
Vx2, Vy2 = 0,0
x2_data, y2_data = [x2], [y2]

def Hopf(x, y, Vx, Vy, x_data, y_data):
    for _ in range(substep):
        r2 = x**2 + y**2
        Vx = (mu - r2)*x - omega*y
        Vy = (mu - r2)*y + omega*x
        x += Vx * dt
        y += Vy * dt
    
        x_data.append(x)
        y_data.append(y)
    return x, y, Vx1, Vy1, x_data, y_data

fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-6, 6)
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Real-Time Velocity affecting Position")

point1, = ax.plot([],[], "ro", markersize = 8)
point2, = ax.plot([],[], "bo", markersize = 8)
trail1, = ax.plot([],[], "r-", alpha=0.5)
trail2, = ax.plot([],[], "b-", alpha=0.5)

def update(frame):
    global x1, y1, Vx1, Vy1, x1_data, y1_data
    global x2, y2, Vx2, Vy2, x2_data, y2_data

    x1, y1, Vx1, Vy1, x1_data, y1_data = Hopf(x1, y1, Vx1, Vy1, x1_data, y1_data)
    x2, y2, Vx2, Vy2, x2_data, y2_data = Hopf(x2, y2, Vx2, Vy2, x2_data, y2_data)

    point1.set_data([x1], [y1])
    point2.set_data([x2], [y2])

    trail1.set_data(x1_data, y1_data)
    trail2.set_data(x2_data, y2_data)

    return point1, point2, trail1, trail2
ani = animation.FuncAnimation(fig, update, frames=1000, interval=intervl, blit=True, repeat=True)

plt.show()