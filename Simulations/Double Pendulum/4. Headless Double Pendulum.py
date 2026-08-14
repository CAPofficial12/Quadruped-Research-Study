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
    v = np.zeros(6)

    data.qpos[0] = q1 + np.pi/2
    data.qpos[1] = q2

    data.qvel[0] = 0.0
    data.qvel[1] = 0.0

    while data.time < duration:

        mujoco.mj_step(model, data)

        mujoco.mj_objectVelocity(
                model,
                data,
                mujoco.mjtObj.mjOBJ_BODY,
                weightID,
                v,
                0
            )
        vx, vy, vz = v[0:3]
        speed = np.linalg.norm(v[0:3])

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
    

    csv_path = script_dir / "Results" / f"Double Pendulum. Test {test_num}.csv"
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

t = 0
for q1 in np.arange(0.0,0.1, 0.001):
    for q2 in np.arange(0.0,0.1, 0.001):
        t += 1
        run_sim(t, q1, q2, 30)
        print("Completed Test: ", t)