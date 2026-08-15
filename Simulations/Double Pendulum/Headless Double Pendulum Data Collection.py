import mujoco
import time
import numpy as np
import csv
from pathlib import Path

def create_xml(g, m, l1, l2, c1, c2):
    script_dir = Path(__file__).resolve().parent
    template_path = script_dir / "Double_template.xml"

    with open(template_path, "r", encoding="utf-8") as file:
        xml = file.read()

    xml = xml.replace("{G}", str(g))
    xml = xml.replace("{L1}", str(l1))
    xml = xml.replace("{L2}", str(l2))
    xml = xml.replace("{M}", str(m))
    xml = xml.replace("{C1}", str(c1))
    xml = xml.replace("{C2}", str(c2))

    test_xml_path = script_dir / "Double.xml"

    with open(test_xml_path, "w", encoding="utf-8") as file:
        file.write(xml)

def run_sim(test_num, g, m, l1, l2 , c1, c2, q1, q2, duration):

    script_dir = Path(__file__).resolve().parent
    xml_fullpath = script_dir / f"Double.xml"
    create_xml(g, m, l1, l2, c1, c2)
    model = mujoco.MjModel.from_xml_path(str(xml_fullpath))
    data = mujoco.MjData(model)

    weightID = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "weight"
    )
    TopID = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "top_hinge"
    )
    BottomID = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "bottom_hinge"
    )
    TopJointID = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "link1"
    )
    BottomJointID = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "link2"
    )

    result = []
    vel = np.zeros(6)
    acc = np.zeros(6)

    data.qpos[0] = q1
    data.qpos[1] = q2

    data.qvel[0] = 0.0
    data.qvel[1] = 0.0

    mujoco.mj_forward(model, data)

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
                g,
                m,
                l1,
                l2,
                c1,
                c2,
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
    csv_fullpath = script_dir / "Full Mass Results" / f"Double Pendulum. Test {test_num}.csv"
    with open(csv_fullpath, "w", newline="\n") as file:
        writer = csv.writer(file)

        writer.writerow([
            "time",
            "gravity",
            "weight_mass",
            "q1_length",
            "q2_length",
            "q1_dampening",
            "q2_dampening",
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
for mass in np.arange(0, 10, 0.01):
    t += 1
    run_sim(t, 9.81, mass, 1, 1, 0.01, 0.01, np.pi/2, np.pi/2, 15)
    print("Completed Test", t, end="; ")
    print("Mass =", mass)