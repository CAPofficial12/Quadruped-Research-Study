import mujoco
import time
import numpy as np
import csv
from pathlib import Path

script_dir = Path(__file__).resolve().parent

def run_sim(test_num, gravity, duration):

    xml_path = script_dir / "Double.xml"

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    model.opt.gravity[:] = [0, 0, -gravity]
    weightID = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "weight"
    )
    result = []
    vel = np.zeros(6)
    acc = np.zeros(6)

    data.qpos[0] = np.pi/2
    data.qpos[1] = np.pi/2

    data.qvel[0] = 0.0
    data.qvel[1] = 0.0

    while data.time < duration:

        mujoco.mj_step(model, data)

        x, y, z = data.xpos[weightID].copy()
        mujoco.mj_objectVelocity(
                model,
                data,
                mujoco.mjtObj.mjOBJ_BODY,
                weightID,
                vel,
                0
        )

        mujoco.mj_objectAcceleration(
            model,
            data,
            mujoco.mjtObj.mjOBJ_BODY,
            weightID,
            acc,
            0
        )
        
        mujoco.mj_energyVel(model, data)
        mujoco.mj_energyPos(model, data)
        

        result.append([
                data.time,
                gravity,
                data.qpos[0],
                data.qpos[1],
                data.qvel[0],
                data.qvel[1],
                data.qacc[0],
                data.qacc[1],
                x,
                y,
                z,
                vel[3],
                vel[4],
                vel[5],
                acc[3],
                acc[4],
                acc[5],
                data.energy[1],
                data.energy[0],
                data.energy[0] + data.energy[1]
            ])
    

    #Adds Full results run timestamp results to Folder
    csv_fullpath = script_dir / "Full Gravity Results" / f"Double Pendulum. Test {test_num}.csv"
    with open(csv_fullpath, "w", newline="\n") as file:
        writer = csv.writer(file)

        writer.writerow([
            "time",
            "gravity",
            "q1",
            "q2",
            "q1_dot",
            "q2_dot",
            "q1_ddot",
            "q2_ddot",
            "weight_x",
            "weight_y",
            "weight_z",
            "weight_vx",
            "weight_vy",
            "weight_vz",
            "weight_ax",
            "weight_ay",
            "weight_az",
            "kinetic energy",
            "potential energy",
            "total energy"
        ])

        writer.writerows(result)

t = 0
for gravity in np.arange(0.0,10, 0.01):
        t += 1
        run_sim(t, gravity, 15)
        print("Completed Test: ", t)