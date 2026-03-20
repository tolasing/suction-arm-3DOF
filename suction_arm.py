import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("./suction_arm/scene.xml")
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)
