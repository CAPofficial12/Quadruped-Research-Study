import mujoco
import numpy as np
import matplotlib.pyplot as plt
import time


# ============================================================
# 1. DOUBLE PENDULUM MJCF
# ============================================================

xml_model = """
<mujoco model="double_pendulum">

    <option
        timestep="0.001"
        gravity="0 0 -9.81"
    />

    <worldbody>

        <!-- First pendulum -->
        <body name="upper_pendulum" pos="0 0 2">

            <!-- First joint -->
            <joint
                name="joint1"
                type="hinge"
                axis="0 1 0"
                damping="0.0"
            />

            <!-- First rod -->
            <geom
                name="rod1"
                type="capsule"
                fromto="0 0 0 0 0 -1"
                size="0.05"
                mass="1"
            />

            <!-- Second pendulum -->
            <body name="lower_pendulum" pos="0 0 -1">

                <!-- Second joint -->
                <joint
                    name="joint2"
                    type="hinge"
                    axis="0 1 0"
                    damping="0.0"
                />

                <!-- Second rod -->
                <geom
                    name="rod2"
                    type="capsule"
                    fromto="0 0 0 0 0 -1"
                    size="0.05"
                    mass="1"
                />

                <!-- Endpoint -->
                <site
                    name="pendulum_end"
                    pos="0 0 -1"
                    size="0.05"
                />

            </body>

        </body>

    </worldbody>

</mujoco>
"""


# ============================================================
# 2. CREATE MODEL
# ============================================================

model = mujoco.MjModel.from_xml_string(xml_model)

data = mujoco.MjData(model)


# ============================================================
# 3. FIND ENDPOINT
# ============================================================

site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "pendulum_end"
)


# ============================================================
# 4. EXPERIMENT PARAMETERS
# ============================================================

NUMBER_OF_RUNS = 2

SIMULATION_TIME = 60.0

DT = 0.001

# Record every 10 physics steps
RECORD_EVERY = 10

# Therefore recorded timestep = 0.01 seconds
RECORD_DT = DT * RECORD_EVERY


# Initial conditions

INITIAL_THETA_1 = 120.0

INITIAL_THETA_2 = -20.0

# Difference between runs

ANGLE_STEP = 0.02


# ============================================================
# 5. NUMBER OF PHYSICS STEPS
# ============================================================

TOTAL_STEPS = int(
    SIMULATION_TIME / DT
)

RECORD_COUNT = TOTAL_STEPS // RECORD_EVERY


print("=" * 60)

print("DOUBLE PENDULUM CHAOS EXPERIMENT")

print("=" * 60)

print(f"Number of runs:       {NUMBER_OF_RUNS}")

print(f"Simulation time:      {SIMULATION_TIME} s")

print(f"Physics timestep:     {DT} s")

print(f"Recorded timestep:    {RECORD_DT} s")

print(f"Physics steps/run:    {TOTAL_STEPS:,}")

print(f"Initial angle:        {INITIAL_THETA_1}°")

print(f"Angle difference:     {ANGLE_STEP}°")

print("=" * 60)


# ============================================================
# 6. STORAGE
# ============================================================

# Shape:
#
# [run][time]
#
# all_x[3][1000]
# means X position of run 4 at timestep 1001

all_x = np.zeros(
    (NUMBER_OF_RUNS, RECORD_COUNT)
)

all_z = np.zeros(
    (NUMBER_OF_RUNS, RECORD_COUNT)
)


# Time vector

time_data = np.arange(
    RECORD_COUNT
) * RECORD_DT


# ============================================================
# 7. RUN EXPERIMENT
# ============================================================

experiment_start = time.perf_counter()


for run in range(NUMBER_OF_RUNS):

    run_start = time.perf_counter()


    # --------------------------------------------------------
    # Reset MuJoCo
    # --------------------------------------------------------

    mujoco.mj_resetData(
        model,
        data
    )


    # --------------------------------------------------------
    # Set initial conditions
    # --------------------------------------------------------

    theta_1 = (
        INITIAL_THETA_1
        + run * ANGLE_STEP
    )

    theta_2 = INITIAL_THETA_2


    data.qpos[0] = np.radians(theta_1)

    data.qpos[1] = np.radians(theta_2)


    # Initial angular velocities

    data.qvel[0] = 0.0

    data.qvel[1] = 0.0


    # Update MuJoCo state

    mujoco.mj_forward(
        model,
        data
    )


    # --------------------------------------------------------
    # Run simulation
    # --------------------------------------------------------

    record_index = 0


    for step in range(TOTAL_STEPS):

        mujoco.mj_step(
            model,
            data
        )


        # Record only every RECORD_EVERY steps

        if step % RECORD_EVERY == 0:

            position = data.site_xpos[
                site_id
            ]


            all_x[
                run,
                record_index
            ] = position[0]


            all_z[
                run,
                record_index
            ] = position[2]


            record_index += 1


    # --------------------------------------------------------
    # Progress information
    # --------------------------------------------------------

    run_time = (
        time.perf_counter()
        - run_start
    )

    total_time = (
        time.perf_counter()
        - experiment_start
    )


    print(
        f"Run {run + 1:2d}/{NUMBER_OF_RUNS} | "
        f"θ₁ = {theta_1:.3f}° | "
        f"runtime = {run_time:.2f}s | "
        f"total = {total_time:.2f}s"
    )


# ============================================================
# 8. EXPERIMENT COMPLETE
# ============================================================

total_runtime = (
    time.perf_counter()
    - experiment_start
)


print("=" * 60)

print("EXPERIMENT COMPLETE")

print(
    f"Total computation time: "
    f"{total_runtime:.2f} seconds"
)

print(
    f"Simulated time: "
    f"{NUMBER_OF_RUNS * SIMULATION_TIME:.1f} seconds"
)

print(
    f"Simulation speed: "
    f"{(NUMBER_OF_RUNS * SIMULATION_TIME) / total_runtime:.1f}x real time"
)

print("=" * 60)

# ============================================================
# 9. INTERACTIVE TRAJECTORY PLOT
# ============================================================

fig, ax = plt.subplots(figsize=(9, 9))

# ------------------------------------------------------------
# Create all trajectory lines
# ------------------------------------------------------------

trajectory_lines = []

for run in range(NUMBER_OF_RUNS):

    line, = ax.plot(
        all_x[run],
        all_z[run],
        linewidth=1.0,
        alpha=0.35,
        picker=5
    )

    trajectory_lines.append(line)


# ------------------------------------------------------------
# Starting point
# ------------------------------------------------------------

ax.plot(
    all_x[0, 0],
    all_z[0, 0],
    "go",
    markersize=8,
    label="Initial position"
)


# ------------------------------------------------------------
# Marker showing the selected trajectory
# ------------------------------------------------------------

selected_point, = ax.plot(
    [],
    [],
    "o",
    markersize=9,
    visible=False
)


# ------------------------------------------------------------
# Information box
# ------------------------------------------------------------

info_text = ax.text(
    0.02,
    0.98,
    "",
    transform=ax.transAxes,
    verticalalignment="top",
    fontsize=11,
    visible=False,
    bbox=dict(
        boxstyle="round",
        facecolor="white",
        alpha=0.85
    )
)


# ------------------------------------------------------------
# Plot formatting
# ------------------------------------------------------------

ax.set_xlabel(
    "X position (m)"
)

ax.set_ylabel(
    "Z position (m)"
)

ax.set_title(
    "Double Pendulum — Interactive Chaotic Trajectories"
)

ax.axis("equal")

ax.grid(True)

ax.legend()


# ============================================================
# 10. MOUSE INTERACTION
# ============================================================

selected_run = None


def find_closest_trajectory(event):

    """
    Find the trajectory closest to the mouse cursor.
    """

    if event.inaxes != ax:
        return None


    # Convert mouse position to data coordinates

    mouse_x = event.xdata
    mouse_z = event.ydata


    # --------------------------------------------------------
    # Find closest point on every trajectory
    # --------------------------------------------------------

    minimum_distance = float("inf")

    closest_run = None


    for run in range(NUMBER_OF_RUNS):

        dx = all_x[run] - mouse_x

        dz = all_z[run] - mouse_z

        distance_squared = dx**2 + dz**2


        index = np.argmin(
            distance_squared
        )


        distance = distance_squared[index]


        if distance < minimum_distance:

            minimum_distance = distance

            closest_run = run


    # --------------------------------------------------------
    # Only select if cursor is reasonably close
    # --------------------------------------------------------

    # Distance threshold in metres

    selection_radius = 0.08


    if minimum_distance < selection_radius**2:

        return closest_run


    return None


# ============================================================
# 11. MOUSE MOVEMENT
# ============================================================

def on_mouse_move(event):

    global selected_run


    closest_run = find_closest_trajectory(event)


    # --------------------------------------------------------
    # Cursor is not near any trajectory
    # --------------------------------------------------------

    if closest_run is None:

        if selected_run is not None:

            selected_run = None

            for line in trajectory_lines:

                line.set_alpha(0.35)

                line.set_linewidth(1.0)


            selected_point.set_visible(False)

            info_text.set_visible(False)

            fig.canvas.draw_idle()


        return


    # --------------------------------------------------------
    # Cursor is near a trajectory
    # --------------------------------------------------------

    selected_run = closest_run


    # --------------------------------------------------------
    # Fade all other trajectories
    # --------------------------------------------------------

    for run, line in enumerate(
        trajectory_lines
    ):

        if run == selected_run:

            line.set_alpha(1.0)

            line.set_linewidth(3.0)

        else:

            line.set_alpha(0.15)

            line.set_linewidth(0.8)


    # --------------------------------------------------------
    # Find closest point on selected trajectory
    # --------------------------------------------------------

    dx = (
        all_x[selected_run]
        - event.xdata
    )

    dz = (
        all_z[selected_run]
        - event.ydata
    )


    index = np.argmin(
        dx**2 + dz**2
    )


    # --------------------------------------------------------
    # Move marker
    # --------------------------------------------------------

    selected_point.set_data(
        [all_x[selected_run, index]],
        [all_z[selected_run, index]]
    )

    selected_point.set_visible(True)


    # --------------------------------------------------------
    # Information
    # --------------------------------------------------------

    initial_angle = (
        INITIAL_THETA_1
        + selected_run * ANGLE_STEP
    )


    info_text.set_text(
        f"Run: {selected_run + 1}\n"
        f"Initial θ₁: {initial_angle:.3f}°\n"
        f"Initial θ₂: {INITIAL_THETA_2:.3f}°\n"
        f"Time: {time_data[index]:.2f} s\n"
        f"X: {all_x[selected_run, index]:.3f} m\n"
        f"Z: {all_z[selected_run, index]:.3f} m"
    )


    info_text.set_visible(True)


    fig.canvas.draw_idle()


# ============================================================
# 12. MOUSE CLICK
# ============================================================

locked_run = None


def on_click(event):

    global locked_run


    if event.inaxes != ax:

        return


    closest_run = find_closest_trajectory(event)


    if closest_run is None:

        return


    # --------------------------------------------------------
    # Lock the selected trajectory
    # --------------------------------------------------------

    if locked_run == closest_run:

        locked_run = None

    else:

        locked_run = closest_run


# ============================================================
# 13. CONNECT MOUSE EVENTS
# ============================================================

fig.canvas.mpl_connect(
    "motion_notify_event",
    on_mouse_move
)

fig.canvas.mpl_connect(
    "button_press_event",
    on_click
)


# ============================================================
# 14. SHOW
# ============================================================

plt.tight_layout()

plt.show()