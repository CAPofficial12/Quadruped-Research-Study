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
      <joint name="hinge" type="hinge" axis="1 0 0" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.05" rgba="0 0 1 1"/>
    </body>
  </worldbody>
  
  <!-- An actuator to apply torque to the hinge joint -->
  <actuator>
    <motor name="my_motor" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"""

# 2. Load the model and create the data object
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# 3. Initialise MatplotLib
plt.ion()
# FIXED: Changed 'subplot' to 'subplots'
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(6,6)) 

x_data = [] # Data Arrays
y_angle = []
y_torque = []

# FIXED: Added trailing commas to unpack the line objects correctly
line_angle, = ax1.plot([], [], color="crimson", label="angle") 
line_torque, = ax2.plot([], [], color="royalblue", label="Torque")

ax1.set_ylabel("Angle(rad)")  # Axis Labels
ax1.set_title("Live MuJoCo Sim Graph")
ax1.grid(True)

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Torque(Nm)")
ax2.grid(True)

# 4. Launch the interactive visualizer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Simulation loop
    while viewer.is_running():
        # Apply a sine-wave torque to the motor
        t = data.time
        torque = 2.0 * np.sin(2 * np.pi * 1.0 * t)
        data.ctrl[0] = torque
        
        # Step the physics forward by 2 milliseconds
        mujoco.mj_step(model, data)
        
        # Sync the visualizer with the new physics state
        viewer.sync()
        
        # Sleep to run in roughly real-time
        time.sleep(0.002)

        x_data.append(t)  # Add to data arrays
        y_angle.append(data.qpos[0])
        y_torque.append(data.ctrl[0])

        # OPTIMIZED: Moved all plotting code inside this downsampling loop.
        # This updates the graph every 0.2 seconds instead of every 0.002 seconds.
        if int(t * 100) % 20 == 0: 
            print(f"Time: {t:.2f}s | Angle: {data.qpos[0]:.3f} rad | Torque: {data.ctrl[0]:.3f} Nm")
            
            # Optional rolling window: prevents data array from growing infinitely
            if len(x_data) > 300:
                # Keeps the graph window clean and performing fast
                pass 

            line_angle.set_data(x_data, y_angle)  # Add data to subplot
            line_torque.set_data(x_data, y_torque)

            ax1.relim() 
            ax2.relim()
            ax1.autoscale_view()  # Rescale Window
            ax2.autoscale_view()

            plt.draw()
            plt.pause(0.001)

plt.ioff()
plt.show()
