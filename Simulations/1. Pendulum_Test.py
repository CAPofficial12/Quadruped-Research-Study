import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt

# 1. Define the robot in MJCF (XML) format
xml_model = """
<mujoco>
  <worldbody>
    <!-- A light and a floor -->
    <light pos="0 0 3"/>:
    <geom type="plane" size="2 2 0.1"/>
    <!-- The pendulum body -->
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" rgba="0 0 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# 2. Load the model and create the data object
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)
data.qpos[0] = np.pi/2

# 3. Initialise MatplotLib
plt.ion()
# FIXED: Changed to (1, 1) since only ax1 is used
fig, (ax1, ax2) = plt.subplots(2, 1, sharex = True, figsize=(6,6)) 

x_data = [] 
y_angle = []
y_vel = []

# FIXED: Added trailing comma to correctly unpack the Line2D object
line_angle, = ax1.plot([], [], color="crimson", label="angle") 
line_velocity, = ax2.plot([], [], color = "royalblue", label="angle")

ax1.set_ylabel("Angle (rad)")  
ax1.set_title("Live MuJoCo Sim Graph")
ax1.grid(True)

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Angular Velocity (rad/s)")
ax2.grid(True)

flag = False
# 4. Launch the interactive visualizer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[1] = 1.0
    viewer.cam.distance = 2.8
    while viewer.is_running():
        t = data.time
        
        mujoco.mj_step(model, data)
        viewer.sync()

        x_data.append(t)  
        y_angle.append(data.qpos[0])
        y_vel.append(data.qvel[0])

        # Downsample plot updates to keep GUI responsive
        if int(t * 100) % 20 == 0: 
            print(f"Time: {t:.2f}s | Angle: {data.qpos[0]:.3f} rad | Angualr Velocity: {data.qvel[0]:.3f}")

        if abs(data.qvel[0]) < 1e-3 and abs(data.qpos[0])*180/np.pi < 2:
          break
        
    viewer.close()

line_angle.set_data(x_data, y_angle)  
line_velocity.set_data(x_data, y_vel) 
ax1.relim()
ax2.relim()
plt.ioff()
plt.show()
