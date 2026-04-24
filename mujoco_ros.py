import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import mujoco
import mujoco.viewer
import numpy as np
import threading
import os

class SuctionArmSim(Node):
    def __init__(self):
        super().__init__('suction_arm_sim')
        
        # 1. Load MuJoCo Model
        xml_path = os.path.expanduser('~/cad_designs/suction_arm/scene.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 2. ROS Subscriber
        # Listens for an array of 4 floats: [shoulder, deltoid, forearm, wrist]
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'arm_commands',
            self.command_callback,
            10)
        
        # Map actuator names to IDs for easier access
        self.actuator_names = ['shoulder', 'deltoid', 'forearm', 'wrist']
        self.actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) 
                            for name in self.actuator_names]

        self.get_logger().info('MuJoCo Arm Node Started. Listening on /arm_commands')

    def command_callback(self, msg):
        """Update MuJoCo control targets when a ROS message arrives."""
        if len(msg.data) == len(self.actuator_ids):
            for i, val in enumerate(msg.data):
                self.data.ctrl[self.actuator_ids[i]] = val
        else:
            self.get_logger().warn(f"Received {len(msg.data)} commands, but need {len(self.actuator_ids)}")

    def run_sim(self):
        """The main simulation loop (runs in a separate thread)."""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = self.data.time
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                
                # Simple sync to keep simulation timing roughly correct
                import time
                time.sleep(self.model.opt.timestep)

def main(args=None):
    rclpy.init(args=args)
    node = SuctionArmSim()

    # Run MuJoCo in a background thread so rclpy.spin can handle ROS events
    sim_thread = threading.Thread(target=node.run_sim, daemon=True)
    sim_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()