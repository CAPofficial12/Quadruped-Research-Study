import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks

dt = 0.005
fps = 60
substep = max(1, round((1 / fps) / dt))
interval = 1000 / fps

x1, x1_dot, x1_Ddot = -1.0, 0.0, 0.0
y1, y1_dot, y1_Ddot = 5.0, 0.0, 0.0
x0 = [x1, x1_dot]
y0 = [y1, y1_dot]
mu_x, mu_y = 2, 2
end = 200

x_data = []
y_data = []
Graph_SIZE = 7

def VDP_equation(t, y, mu):
    a = y[0]
    a_dot = y[1]
    a_Ddot = mu*(1-a**2)*a_dot - a
    return [a_dot, a_Ddot]

def Solver(a, a_dot, mu, a0):
    t_span = (0, end)
    sol = solve_ivp(VDP_equation, t_span, a0, args = (mu,), t_eval=np.linspace(0, end, 10000))

    a_Ddot = (sol.y[0] - a_dot)/dt
    a_dot = (sol.y[1] - a)/dt
    a = sol.y[0]
    
    return sol.y[0], sol.t

y_data, x_data = Solver(x1, x1_dot, mu_x, x0)

peaks, _ = find_peaks(y_data)
t_peaks = x_data[peaks]
T = np.mean(np.diff(t_peaks))

phase = ((x_data  - t_peaks[0]) % T) / T * 2 * np.pi

cutoff = 4000
plt.polar(phase[cutoff:], y_data[cutoff:], color='darkorange')
plt.show()


#fig, ax = plt.subplots()
#ax.set_xlim(-Graph_SIZE, Graph_SIZE)
#ax.set_ylim(-Graph_SIZE, Graph_SIZE)
#ax.set_aspect("equal", adjustable="box")
#ax.set_xlabel("Knee Angle")
#ax.set_ylabel("Hip Angle")
#ax.set_title("Matsuoka Simulation")
#ax.grid(True, alpha=0.3)
#ax.plot(x_data, y_data, "r-", alpha=0.6, linewidth=1.5, label="Angle movement")