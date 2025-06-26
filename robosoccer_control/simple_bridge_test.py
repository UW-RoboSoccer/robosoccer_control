
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class SimpleBridgeTest(Node):
    def __init__(self):
        super().__init__('simple_bridge_test')
        self.joint_names = [
            "right_elbow", "right_shoulder_roll", "right_shoulder_pitch",
            "left_elbow", "left_shoulder_roll", "left_shoulder_pitch",
            "head_pitch", "head_yaw",
            "right_ankle_pitch", "right_knee", "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "left_ankle_pitch", "left_knee", "left_hip_yaw", "left_hip_roll", "left_hip_pitch"
        ]
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)
        self.create_timer(1.0, self.send_test_command)
        self.get_logger().info("Simple Bridge Test initialized! Will send test trajectory every second.")

    def send_test_command(self):
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.1 * 1e9)
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info(f"Sent test trajectory with {len(self.joint_names)} joints")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBridgeTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 