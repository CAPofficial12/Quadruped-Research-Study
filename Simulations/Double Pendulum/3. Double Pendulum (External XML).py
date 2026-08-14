import mujoco
import mujoco.viewer
import time
import numpy as np
import csv
from pathlib import Path

script_dir = Path(__file__).resolve().parent

duration = 120

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
data.qpos[0] = np.pi/2
data.qpos[1] = np.pi/2
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
        elapsed = time.time() - start_time
        target = data.time
        if target > elapsed:
            time.sleep(target - elapsed)
        if data.time > duration:
            break
        viewer.sync()
    viewer.close()