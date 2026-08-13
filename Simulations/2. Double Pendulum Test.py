import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time


# ============================================================
# Configuration
# ============================================================

SIMULATION_TIME = 30.0

# Pendulum dimensions
L1 = 1.0       # Length of first pendulum
L2 = 1.0       # Length of second pendulum

M1 = 1.0       # Mass of first pendulum
M2 = 1.0       # Mass of second pendulum

G = 9.81

# Initial angles, measured from the downward vertical
THETA1 = np.radians(-120)
THETA2 = np.radians(20)

# Simulation timestep
DT = 0.002


# ============================================================
# MuJoCo model
# ============================================================

xml = f"""
<mujoco model="double_pendulum">

    <compiler angle="radian"/>

    <option timestep="{DT}" gravity="0 0 -{G}"/>

    <visual>
        <global offwidth="1280" offheight="720"/>
    </visual>

    <worldbody>

        <!-- Ground/reference -->
        <geom
            name="ground"
            type="plane"
            pos="0 0 -5"
            size="5 5 0.1"
            rgba="0.15 0.15 0.15 1"
        />

        <!-- Fixed pivot -->
        <body name="pendulum_1" pos="0 0 0">

            <joint
                name="joint_1"
                type="hinge"
                axis="0 1 0"
                pos="0 0 0"
            />

            <!-- First rod -->
            <geom
                name="rod_1"
                type="capsule"
                fromto="0 0 0 {L1} 0 0"
                size="0.035"
                mass="{M1}"
            />

            <!-- First mass -->
            <geom
                name="mass_1"
                type="sphere"
                pos="{L1} 0 0"
                size="0.10"
                mass="{M1}"
            />

            <!-- Second pendulum -->
            <body name="pendulum_2" pos="{L1} 0 0">

                <joint
                    name="joint_2"
                    type="hinge"
                    axis="0 1 0"
                    pos="0 0 0"
                />

                <!-- Second rod -->
                <geom
                    name="rod_2"
                    type="capsule"
                    fromto="0 0 0 {L2} 0 0"
                    size="0.035"
                    mass="{M2}"
                />

                <!-- Second mass -->
                <geom
                    name="mass_2"
                    type="sphere"
                    pos="{L2} 0 0"
                    size="0.12"
                    mass="{M2}"
                />

                <!-- End point -->
                <site
                    name="end"
                    pos="{L2} 0 0"
                    size="0.06"
                    rgba="1 0 0 1"
                />

            </body>

        </body>

    </worldbody>

</mujoco>
"""


# ============================================================
# Create model and data
# ============================================================

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Set initial conditions
data.qpos[0] = THETA1
data.qpos[1] = THETA2

data.qvel[:] = 0

# Forward dynamics once before starting
mujoco.mj_forward(model, data)


# ============================================================
# Storage for trajectory
# ============================================================

trajectory_x = []
trajectory_y = []
trajectory_t = []

start_time = time.time()


# ============================================================
# Run simulation
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # Stop after 30 seconds of simulated time
        if data.time >= SIMULATION_TIME:
            break

        # Advance MuJoCo
        mujoco.mj_step(model, data)

        # ----------------------------------------------------
        # Get position of the end of the second pendulum
        # ----------------------------------------------------

        site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "end"
        )

        end_position = data.site_xpos[site_id]

        x = end_position[0]
        y = end_position[2]

        trajectory_x.append(x)
        trajectory_y.append(y)
        trajectory_t.append(data.time)

        # ----------------------------------------------------
        # Update viewer
        # ----------------------------------------------------

        viewer.sync()

        # Run approximately in real time
        time.sleep(DT * 0.5)


# ============================================================
# Matplotlib trajectory plot
# ============================================================

plt.figure(figsize=(10, 8))

plt.plot(
    trajectory_x,
    trajectory_y,
    linewidth=1.0
)

# Mark starting point
plt.scatter(
    trajectory_x[0],
    trajectory_y[0],
    s=60,
    label="Start"
)

# Mark final point
plt.scatter(
    trajectory_x[-1],
    trajectory_y[-1],
    s=60,
    label="End"
)

# Mark pivot
plt.scatter(
    0,
    0,
    s=80,
    marker="x",
    label="Pivot"
)

plt.xlabel("X position (m)")
plt.ylabel("Z position (m)")
plt.title("Double Pendulum End-Effector Trajectory")

plt.axis("equal")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()