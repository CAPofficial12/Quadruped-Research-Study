import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math as m

dt = 0.005
mu = 1
frequency = 4
omega = 2*m.pi*frequency

fps = 60
substep = int(round((1/fps)/dt))
intervl = int(1000/fps)

x1, y1 = -1,5
k12 = 0
phase12 = m.pi
x1_data, y1_data = [x1], [y1]

x2, y2 = 3,0
k21 = 0.2
phase21 = 0
x2_data, y2_data = [x2], [y2]

current_time = 0
time = [current_time]
initial_dTheta = m.atan2(y1, x1) - m.atan2(y2, x2)
phase_diff = [(initial_dTheta + m.pi) % (2 * m.pi) - m.pi]

def Hopf(x, y, x2, y2,x_data, y_data, c, phase):
    
    r2 = x**2 + y**2
    coupling_x = x2 * m.cos(phase) - y2 * m.sin(phase)
    coupling_y = x2 * m.sin(phase) + y2 * m.cos(phase)
    
    Vx = (mu - r2)*x - omega*y + c * coupling_x
    Vy = (mu - r2)*y + omega*x + c * coupling_y
    x += Vx * dt
    y += Vy * dt

    x_data.append(x)
    y_data.append(y)
    return x, y, x_data, y_data

fig, (ax, ax_phase) = plt.subplots(1, 2)

ax.set_xlim(-10, 10)
ax.set_ylim(-6, 6)
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Real-Time Velocity affecting Position")

point1, = ax.plot([],[], "ro", markersize = 8)
point2, = ax.plot([],[], "bo", markersize = 8)
trail1, = ax.plot([],[], "r-", alpha=0.5)
trail2, = ax.plot([],[], "b-", alpha=0.5)

ax.set_yticks([m.sqrt(mu), -m.sqrt(mu)])
ax.set_xticks([m.sqrt(mu), -m.sqrt(mu)])
ax_phase.grid(True, linestyle="--", alpha=0.6)


ax_phase.set_xlim(0, 20)
ax_phase.set_ylim(-m.pi,m.pi)
ax_phase.set_xlabel("Time (s)")
ax_phase.set_ylabel("Phase Difference (rad)")
ax_phase.set_title("Phase Difference over Time")

ax_phase.set_yticks([-m.pi, -m.pi/2, -m.pi/4, 0, m.pi/4, m.pi/2, m.pi])
ax_phase.grid(True, linestyle="--", alpha=0.6)
phase_line, = ax_phase.plot([], [], "g-", lw=2)


def update(frame):
    global x1, y1, x1_data, y1_data
    global x2, y2, x2_data, y2_data
    global current_time, time, phase_diff

    prev_x1, prev_y1 = x1, y1
    prev_x2, prev_y2 = x2, y2

    x1, y1, x1_data, y1_data = Hopf(x1, y1, prev_x2, prev_y2,x1_data, y1_data, k12, phase12)
    x2, y2, x2_data, y2_data = Hopf(x2, y2, prev_x1, prev_y1,x2_data, y2_data, k21, phase21)

    current_time += substep*dt
    time.append(current_time)

    theta1 = m.atan2(y1, x1)
    theta2 = m.atan2(y2, x2)
    dTheta = theta1 - theta2
    dTheta = (dTheta + m.pi) % (2 * m.pi) - m.pi
    phase_diff.append(dTheta)

    point1.set_data([x1], [y1])
    point2.set_data([x2], [y2])

    trail1.set_data(x1_data, y1_data)
    trail2.set_data(x2_data, y2_data)
    phase_line.set_data(time, phase_diff)

    return point1, point2, trail1, trail2, phase_line
ani = animation.FuncAnimation(fig, update, frames=1000, interval=intervl, blit=True, repeat=True)

plt.show()