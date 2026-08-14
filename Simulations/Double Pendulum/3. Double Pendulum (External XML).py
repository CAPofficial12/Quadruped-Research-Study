import mujoco
import mujoco.viewer
import time
import numpy as np
import csv
from pathlib import Path

script_dir = Path(__file__).resolve().parent

def run_sim(test_num, q1, q2, duration):

    xml_path = script_dir / "Double.xml"

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    weightID = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "weight"
    )
    result = []

    data.qpos[0] = q1 + np.pi/2
    data.qpos[1] = q2

    data.qvel[0] = 0.0
    data.qvel[1] = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 1]
        viewer.cam.distance = 7.0
        viewer.cam.azimuth = 0
        viewer.cam.elevation = -10

        start_time = time.time()
        while viewer.is_running():

            mujoco.mj_step(model, data)

            ax, ay, az, vx, vy, vz = data.cvel[weightID]
            speed = (vx**2 + vy**2 + vz**2)**0.5

            result.append([
                data.time,
                data.qpos[0],
                data.qpos[1],
                data.qvel[0],
                data.qvel[1],
                vx,
                vy,
                vz,
                speed
            ])

            elapsed = time.time() - start_time
            target = data.time

            if target > elapsed:
                time.sleep(target - elapsed)

            if data.time > duration:
                break

            viewer.sync()

        viewer.close()
    

    csv_path = script_dir / f"Double Pendulum. Test {test_num}.csv"
    with open(csv_path, "w", newline="\n") as file:
        writer = csv.writer(file)

        writer.writerow([
            "time",
            "q1",
            "q2",
            "q1_dot",
            "q2_dot",
            "weight_x",
            "weight_y",
            "weight_z",
            "weight_dot"
        ])

        writer.writerows(result)
    print("Data exported to CSV File")

tests = [
    (0.1, 0.1),
    (0.2, 0.2),
    (0.3, 0.3),
    (0.5, 0.5),
    (1.0, 1.0)
]

for test_number, (q1, q2) in enumerate(tests, start=1):

    run_sim(
        test_number,
        q1,
        q2,
        duration=10
    )