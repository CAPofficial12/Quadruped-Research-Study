import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

dt = 0.005
mu = 1
frequency = 1
omega = 2*np.pi*frequency
phase = -np.pi/2
alpha = 1
K = 10

t_span = (0, 5)    
t_eval = np.linspace(0, 5, 500)
initial_state = [0.1, 0.0, 0.2, 0.0] 

def Hopf_Vel (t, state):
    Kx,Ky,Hx,Hy = state
    Kr2 = Kx**2 + Ky ** 2
    Hr2 = Hx**2 + Hy ** 2

    dHx = alpha * (mu - Hr2) * Hx - omega * Hy
    dHy = alpha * (mu - Hr2) * Hy + omega * Hx

    Phase_matrix = [[np.cos(phase), -np.sin(phase)],
                    [np.sin(phase),  np.cos(phase)]]

    Ph = np.array([Hx, Hy])
    tH = np.matmul(Phase_matrix, Ph)

    dKx = alpha * (mu - Kr2) * Kx - omega * Ky + K * (tH[0] - Kx)
    dKy = alpha * (mu - Kr2) * Ky + omega * Kx + K * (tH[1] - Ky)
    return [dKx, dKy, dHx, dHy]
sol = solve_ivp(
    Hopf_Vel, 
    t_span, 
    initial_state,
    t_eval=t_eval,
    max_step = dt
)

hip_angles = sol.y[0] 
print(hip_angles)

knee_angles = sol.y[2]
print(knee_angles)
plt.plot(t_eval, hip_angles)
plt.plot(t_eval, knee_angles)
plt.show()
print(f"Simulation completed successfully over {len(sol.t)} time steps.")