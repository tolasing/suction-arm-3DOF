import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')

    def send_goal(self, angles):
        goal_msg = FollowJointTrajectory.Goal()
        point = JointTrajectoryPoint()
        point.positions = angles # e.g., [1.0, 0.5, -0.2, 0.0]
        goal_msg.trajectory.points.append(point)

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: {angles}')
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    client = ArmActionClient()
    # Move shoulder to 1.5 rad and deltoid to 0.7 rad
    future = client.send_goal([1.5, 0.7, 0.0, 0.0])
    rclpy.spin_until_future_complete(client, future)
    print("Goal Accepted/Finished")
    rclpy.shutdown()

if __name__ == '__main__':
    main()