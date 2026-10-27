import numpy as np
import matplotlib.pyplot as plt

# 1. Network and Simulation Parameters
N = 3              
T = 50.0           
dt = 0.01          
steps = int(T / dt)


tau_r = 0.1
tau_a = 1.0
a = 2.5
b = 2.5
s_0 = 1.5          # Tonic drive
K = 0.8            # Kuramoto coupling strength


u1 = np.random.uniform(0.1, 0.5, N)
f1 = np.zeros(N)
u2 = np.random.uniform(0.1, 0.5, N)
f2 = np.zeros(N)

# Storage for plotting
history_u1 = np.zeros((steps, N))
history_R = np.zeros(steps)

for step in range(steps):
    y1 = np.maximum(0, u1)
    y2 = np.maximum(0, u2)
    
    theta = np.arctan2(u2, u1)
    
    complex_phases = np.exp(1j * theta)
    Z = np.mean(complex_phases)
    R = np.abs(Z)
    psi = np.angle(Z)
    
    C = K * R * np.sin(psi - theta)
    
    du1 = (-u1 - a * y2 - b * f1 + s_0 + C*u2) / tau_r
    df1 = (-f1 + y1) / tau_a
    
    du2 = (-u2 - a * y1 - b * f2 + s_0 + C*u1) / tau_r
    df2 = (-f2 + y2) / tau_a
    
    # Update states
    u1 += du1 * dt
    f1 += df1 * dt
    u2 += du2 * dt
    f2 += df2 * dt
    
    # Save states
    history_u1[step, :] = u1
    history_R[step] = R

time_axis = np.linspace(0, T, steps)

plt.figure(figsize=(12, 5))
plt.subplot(2, 1, 1)
plt.plot(time_axis, history_u1)
plt.title("Matsuoka Membrane Potentials ($u_1$) under Kuramoto Coupling")
plt.ylabel("$u_{i,1}$")
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(time_axis, history_R, color='black', lw=2)
plt.title("Kuramoto Order Parameter ($R$) Over Time (1.0 = Perfect Sync)")
plt.xlabel("Time (s)")
plt.ylabel("Coherence ($R$)")
plt.ylim(0, 1.05)
plt.grid(True)

plt.tight_layout()
plt.show()
