import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

y1, y1_dot, y1_Ddot = 3.0, 0.0, 0.0
y0 = [y1, y1_dot]
mu_y = 0.5
end = 50
dt = end/1000

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
    sol = solve_ivp(VDP_equation, t_span, a0, args = (mu,), t_eval=np.linspace(0, end, 1000))

    a_Ddot = (sol.y[0] - a_dot)/dt
    a_dot = (sol.y[1] - a)/dt
    a = sol.y[0]
    
    return sol.y[0]

y_data = Solver(y1, y1_dot, mu_y, y0)
x_data = np.linspace(0, end, 1000)

fig, ax = plt.subplots()

ax.set_xlim(0, end)
ax.set_ylim(-Graph_SIZE, Graph_SIZE)
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("TIme (s)")
ax.set_ylabel("Oscillator State")
ax.set_title("VDP Simulation")
ax.grid(True, alpha=0.3)

ax.plot(x_data, y_data, "r-", alpha=0.6, linewidth=1.5, label="Oscillator Change over time")

ax.legend()
plt.show()