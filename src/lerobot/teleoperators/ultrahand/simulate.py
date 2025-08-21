import mujoco
import mujoco.viewer

from .ultrahand import Ultrahand
from .config_ultrahand import UltrahandConfig


class UltrahandSimulator:
    def __init__(self, port: str, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.ultrahand = Ultrahand(config=UltrahandConfig(port=port))
        self.ultrahand.connect()

    def run_viewer(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                positions = self.ultrahand.get_motors_position()
                self.data.qpos[:7] = positions
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
