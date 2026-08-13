import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt


# ============================================================
# 1. Define the double pendulum in MJCF
# ============================================================

xml_model = """
<mujoco model="double_pendulum">

    <option timestep="0.002" gravity="0 0 -9.81"/>

    <visual>
        <headlight diffuse="0.8 0.8 0.8"/>
        <rgba haze="0.15 0.15 0.15 1"/>
    </visual>

    <worldbody>

        <!-- Ground -->
        <geom
            name="ground"
            type="plane"
            pos="0 0 -2"
            size="5 5 0.1"
            rgba="0.7 0.7 0.7 1"
        />

        <!-- Light -->
        <light
            pos="0 -2 4"
            directional="false"
        />

        <!-- Fixed pivot -->
        <body name="upper_pendulum" pos="0 0 2">

            <!-- First joint -->
            <joint
                name="joint1"
                type="hinge"
                axis="0 1 0"
                damping="0.02"
            />

            <!-- First pendulum rod -->
            <geom
                name="rod1"
                type="capsule"
                fromto="0 0 0 0 0 -1"
                size="0.06"
                mass="1"
                rgba="0.1 0.3 0.9 1"
            />

            <!-- Joint between first and second links -->
            <body name="lower_pendulum" pos="0 0 -1">

                <joint
                    name="joint2"
                    type="hinge"
                    axis="0 1 0"
                    damping="0.02"
                />

                <!-- Second pendulum rod -->
                <geom
                    name="rod2"
                    type="capsule"
                    fromto="0 0 0 0 0 -1"
                    size="0.06"
                    mass="1"
                    rgba="0.9 0.2 0.2 1"
                />

                <!-- Endpoint used for trajectory tracking -->
                <site
                    name="pendulum_end"
                    pos="0 0 -1"
                    size="0.10"
                    rgba="0.2 1 0.2 1"
                />

            </body>
        </body>

    </worldbody>

</mujoco>
"""


# ============================================================
# 2. Load MuJoCo model
# ============================================================

model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)


# ============================================================
# 3. Initial conditions
# ============================================================

# First pendulum angle
data.qpos[0] = np.radians(120)

# Second pendulum angle
data.qpos[1] = np.radians(-20)

# Initial angular velocities
data.qvel[0] = 0.0
data.qvel[1] = 0.0

# Forward dynamics so MuJoCo knows the initial positions
mujoco.mj_forward(model, data)


# ============================================================
# 4. Find the endpoint site
# ============================================================

site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "pendulum_end"
)


# ============================================================
# 5. Data for trajectory
# ============================================================

x_data = []
z_data = []
time_data = []


# ============================================================
# 6. Matplotlib setup
# ============================================================

plt.ion()

fig, ax = plt.subplots(figsize=(7, 7))

line, = ax.plot(
    [],
    [],
    linewidth=1.5,
    label="End-point trajectory"
)

point, = ax.plot(
    [],
    [],
    "o",
    markersize=6,
    label="Pendulum end"
)

ax.set_title("Double Pendulum Trajectory")
ax.set_xlabel("X position (m)")
ax.set_ylabel("Z position (m)")

ax.set_xlim(-4, 4)
ax.set_ylim(-4, -1)

ax.set_aspect("equal")
ax.grid(True)
ax.legend()


# ============================================================
# 7. Run MuJoCo simulation
# ============================================================

simulation_duration = 30.0

with mujoco.viewer.launch_passive(model, data) as viewer:

    # Camera setup
    viewer.cam.lookat[:] = [0, 0, 1]
    viewer.cam.distance = 4.5
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -10

    start_time = time.perf_counter()

    while viewer.is_running():

        # ----------------------------------------------------
        # Advance simulation
        # ----------------------------------------------------

        mujoco.mj_step(model, data)

        # ----------------------------------------------------
        # Get endpoint position
        # ----------------------------------------------------

        position = data.site_xpos[site_id]

        x = position[0]
        z = position[2]

        t = data.time

        # ----------------------------------------------------
        # Store trajectory
        # ----------------------------------------------------

        x_data.append(x)
        z_data.append(z)
        time_data.append(t)

        # ----------------------------------------------------
        # Update MuJoCo viewer
        # ----------------------------------------------------

        viewer.sync()

        # ----------------------------------------------------
        # Update Matplotlib occasionally
        # ----------------------------------------------------

        if len(x_data) % 20 == 0:

            line.set_data(x_data, z_data)
            point.set_data([x], [z])

            fig.canvas.draw_idle()
            fig.canvas.flush_events()

        # ----------------------------------------------------
        # Run in approximately real time
        # ----------------------------------------------------

        elapsed = time.perf_counter() - start_time

        if t > elapsed:
            time.sleep(t - elapsed)

        # ----------------------------------------------------
        # Stop after 30 seconds
        # ----------------------------------------------------

        if t >= simulation_duration:
            break


# ============================================================
# 8. Final trajectory plot
# ============================================================

plt.ioff()

fig2, ax2 = plt.subplots(figsize=(8, 8))

ax2.plot(
    x_data,
    z_data,
    linewidth=1.2
)

# Starting point
ax2.plot(
    x_data[0],
    z_data[0],
    "go",
    label="Start"
)

# Final point
ax2.plot(
    x_data[-1],
    z_data[-1],
    "ro",
    label="End"
)

ax2.set_title(
    "Double Pendulum End-Point Trajectory — 30 Seconds"
)

ax2.set_xlabel("X position (m)")
ax2.set_ylabel("Z position (m)")

ax2.set_aspect("equal")
ax2.grid(True)
ax2.legend()

plt.show()