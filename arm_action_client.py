import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
import os

# Note: In a real project, you'd define a .action file. 
# For this example, we'll use a built-in or mock structure.
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MuJoCoActionServer(Node):
    def __init__(self):
        super().__init__('mujoco_action_server')
        
        # 1. Load MuJoCo
        xml_path = os.path.expanduser('~/cad_designs/suction_arm/scene.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 2. Setup Action Server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            self.execute_callback)

        self.get_logger().info("MuJoCo Action Server is ready.")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_positions = goal_handle.request.trajectory.points[-1].positions
        
        # 1. Update MuJoCo Control
        for i in range(min(len(target_positions), 4)):
            self.data.ctrl[i] = target_positions[i]

        feedback_msg = FollowJointTrajectory.Feedback()
        
        # 2. Monitor Loop
        for _ in range(200):
            # Explicitly cast to float to satisfy ROS 2 type checking
            current_positions = [float(self.data.actuator(i).length) for i in range(4)]
            
            # Send feedback
            feedback_msg.actual.positions = current_positions
            goal_handle.publish_feedback(feedback_msg)

            # Calculation for error (NumPy is fine here)
            error = np.linalg.norm(np.array(target_positions[:4]) - np.array(current_positions))
            
            if error < 0.01:
                break
            time.sleep(0.01)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result
    def run_sim(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(self.model.opt.timestep)

def main():
    rclpy.init()
    server = MuJoCoActionServer()
    threading.Thread(target=server.run_sim, daemon=True).start()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()