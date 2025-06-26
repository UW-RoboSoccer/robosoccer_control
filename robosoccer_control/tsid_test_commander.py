import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robosoccer_control.msg import TSIDCommand
import math

class TSIDTestCommander(Node):
    def __init__(self):
        super().__init__('tsid_test_commander')
        self.declare_parameter('test_mode', 'standing')
        self.declare_parameter('command_frequency', 10.0)
        self.test_mode = self.get_parameter('test_mode').get_parameter_value().string_value
        self.command_frequency = self.get_parameter('command_frequency').get_parameter_value().double_value
        self.tsid_command_pub = self.create_publisher(TSIDCommand, '/tsid_commands', 10)
        self.com_target_pub = self.create_publisher(PoseStamped, '/com_target', 10)
        self.left_foot_target_pub = self.create_publisher(PoseStamped, '/left_foot_target', 10)
        self.right_foot_target_pub = self.create_publisher(PoseStamped, '/right_foot_target', 10)
        self.test_time = 0.0
        self.phase = 0
        self.create_timer(1.0 / self.command_frequency, self.command_loop)
        self.get_logger().info(f"TSID Test Commander initialized! Test mode: {self.test_mode}")

    def command_loop(self):
        self.test_time += 1.0 / self.command_frequency
        if self.test_mode == 'standing':
            self.send_standing_command()
        elif self.test_mode == 'com_tracking':
            self.send_com_tracking_command()
        elif self.test_mode == 'foot_tracking':
            self.send_foot_tracking_command()
        elif self.test_mode == 'simple_motion':
            self.send_simple_motion_command()

    def send_standing_command(self):
        tsid_cmd = TSIDCommand()
        tsid_cmd.desired_com_pose.position.x = 0.0
        tsid_cmd.desired_com_pose.position.y = 0.0
        tsid_cmd.desired_com_pose.position.z = 0.4
        tsid_cmd.desired_com_pose.orientation.w = 1.0
        tsid_cmd.desired_com_velocity.linear.x = 0.0
        tsid_cmd.desired_com_velocity.linear.y = 0.0
        tsid_cmd.desired_com_velocity.linear.z = 0.0
        tsid_cmd.left_foot_contact = True
        tsid_cmd.right_foot_contact = True
        self.tsid_command_pub.publish(tsid_cmd)
        self.get_logger().info_throttle(2.0, "Sending standing command - CoM at (0, 0, 0.4)")

    def send_com_tracking_command(self):
        tsid_cmd = TSIDCommand()
        radius = 0.05
        frequency = 0.5
        tsid_cmd.desired_com_pose.position.x = radius * math.cos(2 * math.pi * frequency * self.test_time)
        tsid_cmd.desired_com_pose.position.y = radius * math.sin(2 * math.pi * frequency * self.test_time)
        tsid_cmd.desired_com_pose.position.z = 0.4
        tsid_cmd.desired_com_pose.orientation.w = 1.0
        tsid_cmd.desired_com_velocity.linear.x = -radius * 2 * math.pi * frequency * math.sin(2 * math.pi * frequency * self.test_time)
        tsid_cmd.desired_com_velocity.linear.y = radius * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * self.test_time)
        tsid_cmd.desired_com_velocity.linear.z = 0.0
        tsid_cmd.left_foot_contact = True
        tsid_cmd.right_foot_contact = True
        self.tsid_command_pub.publish(tsid_cmd)
        self.get_logger().info_throttle(2.0, "Sending CoM tracking command - Circular motion")

    def send_foot_tracking_command(self):
        left_foot_msg = PoseStamped()
        right_foot_msg = PoseStamped()
        if self.phase < 100:
            right_foot_msg.pose.position.z = 0.05 * math.sin(math.pi * self.test_time)
            left_foot_msg.pose.position.z = 0.0
        elif self.phase < 200:
            right_foot_msg.pose.position.z = 0.0
            left_foot_msg.pose.position.z = 0.05 * math.sin(math.pi * self.test_time)
        else:
            self.phase = 0
        self.left_foot_target_pub.publish(left_foot_msg)
        self.right_foot_target_pub.publish(right_foot_msg)
        self.phase += 1
        self.get_logger().info_throttle(2.0, f"Sending foot tracking command - Phase {self.phase}")

    def send_simple_motion_command(self):
        tsid_cmd = TSIDCommand()
        amplitude = 0.02
        frequency = 0.3
        tsid_cmd.desired_com_pose.position.x = amplitude * math.sin(2 * math.pi * frequency * self.test_time)
        tsid_cmd.desired_com_pose.position.y = 0.0
        tsid_cmd.desired_com_pose.position.z = 0.4
        tsid_cmd.desired_com_pose.orientation.w = 1.0
        tsid_cmd.desired_com_velocity.linear.x = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * self.test_time)
        tsid_cmd.desired_com_velocity.linear.y = 0.0
        tsid_cmd.desired_com_velocity.linear.z = 0.0
        tsid_cmd.left_foot_contact = True
        tsid_cmd.right_foot_contact = True
        self.tsid_command_pub.publish(tsid_cmd)
        self.get_logger().info_throttle(2.0, "Sending simple motion command - Forward/backward")

def main(args=None):
    rclpy.init(args=args)
    node = TSIDTestCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 