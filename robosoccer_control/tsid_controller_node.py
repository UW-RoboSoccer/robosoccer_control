import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from robosoccer_control.msg import TSIDCommand, FootstepArray
import numpy as np
import os
import sys

# Add reference/tsid_control to sys.path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '../../reference/tsid_control'))
from biped import Biped
import op3_conf as default_conf

class TSIDControllerWrapper:
    def __init__(self, conf):
        self.biped = Biped(conf)
        self.conf = conf
        self.q = self.biped.q.copy()
        self.v = self.biped.v.copy()
        self.initialized = True

    def update_robot_state(self, q_full, v_full):
        self.q = q_full.copy()
        self.v = v_full.copy()

    def set_com_reference(self, pos, vel, acc):
        self.biped.trajCom.setReference(np.array(pos))
        self.biped.comTask.setReference(self.biped.trajCom.computeNext())

    def set_posture_reference(self, q_joints):
        self.biped.trajPosture.setReference(np.array(q_joints))
        self.biped.postureTask.setReference(self.biped.trajPosture.computeNext())

    def set_foot_reference(self, foot_name, pose, in_contact):
        import pinocchio as pin
        trans = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = np.array([
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        ])
        SE3 = pin.SE3(pin.Quaternion(quat).matrix(), trans)
        if foot_name == self.conf.lf_frame_name:
            self.biped.trajLF.setReference(SE3)
            self.biped.leftFootTask.setReference(self.biped.trajLF.computeNext())
            if in_contact:
                self.biped.addLeftFootContact()
            else:
                self.biped.removeLeftFootContact()
        elif foot_name == self.conf.rf_frame_name:
            self.biped.trajRF.setReference(SE3)
            self.biped.rightFootTask.setReference(self.biped.trajRF.computeNext())
            if in_contact:
                self.biped.addRightFootContact()
            else:
                self.biped.removeRightFootContact()

    def compute_control_torques(self):
        t = 0.0
        HQPData = self.biped.formulation.computeProblemData(t, self.q, self.v)
        sol = self.biped.solver.solve(HQPData)
        if sol.status != 0:
            return None
        tau = self.biped.formulation.getActuatorForces(sol)
        return tau

class TSIDControllerNode(Node):
    def __init__(self):
        super().__init__('tsid_controller_node')
        self.declare_parameter('robot_description_package', 'my_robot_description')
        self.declare_parameter('urdf_file', 'robot.urdf')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('control_frequency', 1000.0)
        self.declare_parameter('joint_names', [])
        self.conf = default_conf
        self.tsid_controller = TSIDControllerWrapper(self.conf)
        self.joint_names = self.conf.gain_vector.shape[0] if hasattr(self.conf, 'gain_vector') else 16
        self.current_joint_positions = [0.0] * self.joint_names
        self.current_joint_velocities = [0.0] * self.joint_names
        self.posture_target_set = False
        self.joint_commands_pub = self.create_publisher(Float64MultiArray, '/robosoccer/joint_commands', 10)
        self.torque_commands_pub = self.create_publisher(Float64MultiArray, '/computed_torques', 10)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.tsid_command_sub = self.create_subscription(TSIDCommand, '/tsid_commands', self.tsid_command_callback, 10)
        self.footstep_plan_sub = self.create_subscription(FootstepArray, '/motion_plan', self.footstep_plan_callback, 10)
        self.com_target_sub = self.create_subscription(PoseStamped, '/com_target', self.com_target_callback, 10)
        self.left_foot_target_sub = self.create_subscription(PoseStamped, '/left_foot_target', self.left_foot_target_callback, 10)
        self.right_foot_target_sub = self.create_subscription(PoseStamped, '/right_foot_target', self.right_foot_target_callback, 10)
        timer_period = 1.0 / self.conf.dt if hasattr(self.conf, 'dt') else 0.001
        self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('TSID Controller Node initialized!')

    def joint_states_callback(self, msg):
        if len(msg.position) >= len(self.current_joint_positions) and len(msg.velocity) >= len(self.current_joint_velocities):
            self.current_joint_positions = list(msg.position[:len(self.current_joint_positions)])
            self.current_joint_velocities = list(msg.velocity[:len(self.current_joint_velocities)])
            q_full = np.zeros(7 + len(self.current_joint_positions))
            v_full = np.zeros(6 + len(self.current_joint_velocities))
            q_full[:7] = [0, 0, 0.4, 0, 0, 0, 1]
            v_full[:6] = [0, 0, 0, 0, 0, 0]
            q_full[7:] = self.current_joint_positions
            v_full[6:] = self.current_joint_velocities
            self.tsid_controller.update_robot_state(q_full, v_full)
            if not self.posture_target_set:
                self.tsid_controller.set_posture_reference(self.current_joint_positions)
                self.posture_target_set = True

    def tsid_command_callback(self, msg):
        com_pos = [msg.desired_com_pose.position.x, msg.desired_com_pose.position.y, msg.desired_com_pose.position.z]
        com_vel = [msg.desired_com_velocity.linear.x, msg.desired_com_velocity.linear.y, msg.desired_com_velocity.linear.z]
        com_acc = [0.0, 0.0, 0.0]
        self.tsid_controller.set_com_reference(com_pos, com_vel, com_acc)
        self.tsid_controller.set_foot_reference(self.conf.lf_frame_name, msg.left_foot_pose, msg.left_foot_contact)
        self.tsid_controller.set_foot_reference(self.conf.rf_frame_name, msg.right_foot_pose, msg.right_foot_contact)
        if len(msg.joint_positions) == len(self.current_joint_positions):
            self.tsid_controller.set_posture_reference(msg.joint_positions)

    def footstep_plan_callback(self, msg):
        pass

    def com_target_callback(self, msg):
        com_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        com_vel = [0.0, 0.0, 0.0]
        com_acc = [0.0, 0.0, 0.0]
        self.tsid_controller.set_com_reference(com_pos, com_vel, com_acc)

    def left_foot_target_callback(self, msg):
        self.tsid_controller.set_foot_reference(self.conf.lf_frame_name, msg.pose, True)

    def right_foot_target_callback(self, msg):
        self.tsid_controller.set_foot_reference(self.conf.rf_frame_name, msg.pose, True)

    def control_loop(self):
        tau = self.tsid_controller.compute_control_torques()
        if tau is not None:
            torque_msg = Float64MultiArray()
            torque_msg.data = tau.tolist()
            self.torque_commands_pub.publish(torque_msg)
            cmd_msg = Float64MultiArray()
            cmd_msg.data = self.current_joint_positions
            self.joint_commands_pub.publish(cmd_msg)
        else:
            self.get_logger().warn('TSID QP could not be solved!')

def main(args=None):
    rclpy.init(args=args)
    node = TSIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 