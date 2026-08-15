import mujoco
import mujoco.viewer
import time
import numpy as np
import csv
import pandas as pd
from pathlib import Path

test_num = 100
script_dir = Path(__file__).resolve().parent
csv_fullpath = script_dir / "Full Gravity Results" / f"Double Pendulum. Test {test_num}.csv"
df = pd.read_csv(csv_fullpath)
init_state = df.iloc[0]

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

duration = df.iloc[-1]["time"]
data.qpos[0] = init_state["q1"]
data.qpos[1] = init_state["q2"]
data.qvel[0] = init_state["q1_dot"]
data.qvel[1] = init_state["q2_dot"]
model.opt.gravity[:] = [0, 0, -init_state["gravity"]]
model.body_mass[weightID] = 2.0

mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [0, 0, 1]
    
    viewer.cam.distance = 7.0
    viewer.cam.azimuth = 0
    viewer.cam.elevation = -10
    start_time = time.time()
    while viewer.is_running():
        mujoco.mj_step(model, data)
        elapsed = time.time() - start_time
        target = data.time
        if target > elapsed:
            time.sleep(target - elapsed)
        if data.time > duration:
            break
        viewer.sync()
    viewer.close()