import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class TSIDToTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('tsid_to_trajectory_bridge')
        self.declare_parameter('trajectory_duration', 0.05)
        self.declare_parameter('max_velocity', 10.0)
        self.declare_parameter('max_acceleration', 50.0)
        self.trajectory_duration = self.get_parameter('trajectory_duration').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.use_sim_time = self.get_parameter_or('use_sim_time', rclpy.Parameter('use_sim_time', value=False)).value
        self.joint_names = [
            "right_elbow", "right_shoulder_roll", "right_shoulder_pitch",
            "left_elbow", "left_shoulder_roll", "left_shoulder_pitch", 
            "head_pitch", "head_yaw",
            "right_ankle_pitch", "right_knee", "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "left_ankle_pitch", "left_knee", "left_hip_yaw", "left_hip_roll", "left_hip_pitch"
        ]
        self.current_positions = [0.0] * len(self.joint_names)
        self.current_velocities = [0.0] * len(self.joint_names)
        self.last_command_time = self.get_clock().now()
        self.last_joint_state_time = self.get_clock().now()
        self.get_logger().info("Joint orders match exactly - direct 1:1 mapping")
        self.get_logger().info(f"Trajectory duration: {self.trajectory_duration:.3f} s")
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/position_controller/joint_trajectory', 10)
        self.tsid_sub = self.create_subscription(Float64MultiArray, '/robosoccer/joint_commands', 10, self.tsid_command_callback)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', 10, self.joint_states_callback)
        self.status_pub = self.create_publisher(Float64MultiArray, '/bridge_status', 10)
        self.get_logger().info("Enhanced TSID to Trajectory Bridge initialized!")
        self.get_logger().info("Subscribing to: /robosoccer/joint_commands")
        self.get_logger().info("Subscribing to: /joint_states")
        self.get_logger().info("Publishing to: /position_controller/joint_trajectory")
        self.get_logger().info("Publishing status to: /bridge_status")

    def joint_states_callback(self, msg):
        if len(msg.position) >= len(self.joint_names) and len(msg.velocity) >= len(self.joint_names):
            for i in range(len(self.joint_names)):
                self.current_positions[i] = msg.position[i]
                self.current_velocities[i] = msg.velocity[i]
            self.last_joint_state_time = self.get_clock().now()

    def tsid_command_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(f"Received {len(msg.data)} joint commands, expected {len(self.joint_names)}")
            return
        now = self.get_clock().now()
        if (now - self.last_joint_state_time).nanoseconds / 1e9 > 0.1:
            self.get_logger().warn("No recent joint states received. Waiting for joint state feedback.")
            return
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = now.to_msg()
        traj_msg.header.frame_id = ""
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(self.joint_names)
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.effort = []
        point.time_from_start.sec = int(self.trajectory_duration)
        point.time_from_start.nanosec = int((self.trajectory_duration % 1) * 1e9)
        for i in range(len(self.joint_names)):
            target_position = msg.data[i]
            current_position = self.current_positions[i]
            current_velocity = self.current_velocities[i]
            point.positions[i] = target_position
            position_error = target_position - current_position
            desired_velocity = position_error / self.trajectory_duration
            desired_velocity = max(-self.max_velocity, min(self.max_velocity, desired_velocity))
            point.velocities[i] = desired_velocity
            velocity_error = desired_velocity - current_velocity
            desired_acceleration = velocity_error / self.trajectory_duration
            desired_acceleration = max(-self.max_acceleration, min(self.max_acceleration, desired_acceleration))
            point.accelerations[i] = desired_acceleration
        traj_msg.points = [point]
        self.trajectory_pub.publish(traj_msg)
        self.last_command_time = now
        self.publish_status()

    def publish_status(self):
        status_msg = Float64MultiArray()
        status_msg.data = [0.0] * 4
        now = self.get_clock().now()
        status_msg.data[0] = (now - self.last_command_time).nanoseconds / 1e9
        status_msg.data[1] = (now - self.last_joint_state_time).nanoseconds / 1e9
        status_msg.data[2] = self.trajectory_duration
        status_msg.data[3] = float(len(self.joint_names))
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TSIDToTrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 