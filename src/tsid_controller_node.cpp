/*
main node for the TSID controller.
responsible for:
- initializing the TSID controller
- subscribing to the joint states
- publishing the joint commands
- publishing the computed torques


need to test - note for ernest
*/


#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "robosoccer_control/msg/tsid_command.hpp"
#include "robosoccer_control/msg/footstep_array.hpp"
#include "robosoccer_control/tsid_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

class TSIDControllerNode : public rclcpp::Node
{
public:
  TSIDControllerNode() : Node("tsid_controller_node")
  {
    // Declare parameters
    this->declare_parameter("robot_description_package", "my_robot_description");
    this->declare_parameter("urdf_file", "robot.urdf");
    this->declare_parameter("urdf_path", "");  // Direct path override
    this->declare_parameter("control_frequency", 1000.0);
    this->declare_parameter("joint_names", std::vector<std::string>{});
    
    // Get parameters
    std::string robot_pkg = this->get_parameter("robot_description_package").as_string();
    std::string urdf_file = this->get_parameter("urdf_file").as_string();
    std::string direct_urdf_path = this->get_parameter("urdf_path").as_string();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    joint_names_ = this->get_parameter("joint_names").as_string_array();

    // If no joint names provided, use default humanoid joint names
    if (joint_names_.empty()) {
      joint_names_ = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "joint_8",
        "joint_9", "joint_10", "joint_11", "joint_12", "joint_13", "joint_14", "joint_15", "joint_16"
      };
      RCLCPP_WARN(this->get_logger(), "Using default joint names. Configure joint_names parameter for your robot.");
    }

    // Initialize TSID controller
    tsid_controller_ = std::make_unique<robosoccer_control::TSIDController>();
    
    // Build URDF path
    std::string urdf_path;
    
    // If direct path provided, use it
    if (!direct_urdf_path.empty()) {
      urdf_path = direct_urdf_path;
      RCLCPP_INFO(this->get_logger(), "Using direct URDF path: %s", urdf_path.c_str());
    } else {
      // Otherwise try to find package
      try {
        std::string package_path = ament_index_cpp::get_package_share_directory(robot_pkg);
        urdf_path = package_path + "/urdf/" + urdf_file;
        RCLCPP_INFO(this->get_logger(), "Using package URDF path: %s", urdf_path.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find robot description package: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "Try providing direct urdf_path parameter");
        return;
      }
    }

    // Initialize controller with proper TSID config
    robosoccer_control::TSIDConfig tsid_config;
    // Override default config if needed
    tsid_config.kp_com = 100.0;
    tsid_config.kp_foot = 100.0;
    tsid_config.kp_posture = 1.0;
    
    if (!tsid_controller_->initialize(urdf_path, tsid_config)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize TSID controller!");
      return;
    }

    // Initialize joint state
    current_joint_positions_.resize(joint_names_.size(), 0.0);
    current_joint_velocities_.resize(joint_names_.size(), 0.0);

    // Publishers
    joint_commands_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/robosoccer/joint_commands", 10);

    // Optional: Publish computed torques for feedforward
    torque_commands_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/computed_torques", 10);

    // Subscribers
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&TSIDControllerNode::jointStatesCallback, this, std::placeholders::_1));

    // High-level task space command from motion planning
    tsid_command_sub_ = this->create_subscription<robosoccer_control::msg::TSIDCommand>(
      "/tsid_commands", 10,
      std::bind(&TSIDControllerNode::tsidCommandCallback, this, std::placeholders::_1));

    // Motion planning interface
    footstep_plan_sub_ = this->create_subscription<robosoccer_control::msg::FootstepArray>(
      "/motion_plan", 10,
      std::bind(&TSIDControllerNode::footstepPlanCallback, this, std::placeholders::_1));

    // Individual task subscriptions for more granular control
    com_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/com_target", 10,
      std::bind(&TSIDControllerNode::comTargetCallback, this, std::placeholders::_1));

    left_foot_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/left_foot_target", 10,
      std::bind(&TSIDControllerNode::leftFootTargetCallback, this, std::placeholders::_1));

    right_foot_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/right_foot_target", 10,
      std::bind(&TSIDControllerNode::rightFootTargetCallback, this, std::placeholders::_1));

    // Control timer
    auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
      timer_period, std::bind(&TSIDControllerNode::controlLoop, this));

    // Initialize contact constraints (standing by default)
    initializeDefaultContacts();

    RCLCPP_INFO(this->get_logger(), "TSID Controller Node initialized!");
    RCLCPP_INFO(this->get_logger(), "Control frequency: %.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(), "Joint count: %zu", joint_names_.size());
  }

private:
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Update current joint state
    if (msg->position.size() >= joint_names_.size() && 
        msg->velocity.size() >= joint_names_.size()) {
      
      // Map joint states (assuming same order as joint_names_)
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        current_joint_positions_[i] = msg->position[i];
        current_joint_velocities_[i] = msg->velocity[i];
      }

      // Update TSID controller with current state (need full state with free-flyer)
      Eigen::VectorXd q_full = Eigen::VectorXd::Zero(current_joint_positions_.size() + 7);  // +7 for free-flyer
      Eigen::VectorXd v_full = Eigen::VectorXd::Zero(current_joint_velocities_.size() + 6); // +6 for free-flyer
      
      // Set free-flyer to identity for now (standing)
      q_full.head<7>() << 0, 0, 0.4, 0, 0, 0, 1;  // [x,y,z, qx,qy,qz,qw]
      v_full.head<6>().setZero();  // [vx,vy,vz, wx,wy,wz]
      
      // Copy joint states 
      for (size_t i = 0; i < current_joint_positions_.size(); ++i) {
        q_full[7 + i] = current_joint_positions_[i];
        v_full[6 + i] = current_joint_velocities_[i];
      }
      
      tsid_controller_->updateRobotState(q_full, v_full);
      
      // Set current joint pose as posture target if none set
      if (!posture_target_set_) {
        Eigen::VectorXd q_joints = Eigen::Map<Eigen::VectorXd>(current_joint_positions_.data(), current_joint_positions_.size());
        tsid_controller_->setPostureReference(q_joints);
        posture_target_set_ = true;
      }
    }
  }

  void tsidCommandCallback(const robosoccer_control::msg::TSIDCommand::SharedPtr msg)
  {
    // Set CoM target
    robosoccer_control::TaskSpaceTarget com_target;
    com_target.position = Eigen::Vector3d(
      msg->desired_com_pose.position.x,
      msg->desired_com_pose.position.y,
      msg->desired_com_pose.position.z);
    com_target.orientation = Eigen::Quaterniond(
      msg->desired_com_pose.orientation.w,
      msg->desired_com_pose.orientation.x,
      msg->desired_com_pose.orientation.y,
      msg->desired_com_pose.orientation.z);
    com_target.linear_velocity = Eigen::Vector3d(
      msg->desired_com_velocity.linear.x,
      msg->desired_com_velocity.linear.y,
      msg->desired_com_velocity.linear.z);
    com_target.linear_acceleration = Eigen::Vector3d::Zero();
    com_target.weight = 10.0;
    com_target.active = true;

    tsid_controller_->setCoMTarget(com_target);

    // Set foot targets
    setFootTarget("feet", msg->left_foot_pose, msg->left_foot_contact);    // Left foot
    setFootTarget("feet_2", msg->right_foot_pose, msg->right_foot_contact); // Right foot

    // Update posture target if provided
    if (msg->joint_positions.size() == joint_names_.size()) {
      Eigen::VectorXd q_desired = Eigen::Map<const Eigen::VectorXd>(
        msg->joint_positions.data(), msg->joint_positions.size());
      Eigen::VectorXd v_desired = Eigen::VectorXd::Zero(q_desired.size());
      
      if (msg->joint_velocities.size() == joint_names_.size()) {
        v_desired = Eigen::Map<const Eigen::VectorXd>(
          msg->joint_velocities.data(), msg->joint_velocities.size());
      }
      
      tsid_controller_->setPostureTarget(q_desired, v_desired);
    }
  }

  void comTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    robosoccer_control::TaskSpaceTarget com_target;
    com_target.position = Eigen::Vector3d(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    com_target.orientation = Eigen::Quaterniond(
      msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);
    com_target.linear_velocity = Eigen::Vector3d::Zero();
    com_target.linear_acceleration = Eigen::Vector3d::Zero();
    com_target.weight = 10.0;
    com_target.active = true;

    tsid_controller_->setCoMTarget(com_target);
  }

  void leftFootTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    setFootTarget("feet", msg->pose, true);  // Use actual URDF frame name
  }

  void rightFootTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    setFootTarget("feet_2", msg->pose, true);  // Use actual URDF frame name
  }

  void footstepPlanCallback(const robosoccer_control::msg::FootstepArray::SharedPtr msg)
  {
    // Store the current motion plan
    current_motion_plan_ = msg;
    
    // Process current footstep from the plan
    if (!msg->footsteps.empty() && msg->current_step_index < msg->footsteps.size()) {
      const auto& current_step = msg->footsteps[msg->current_step_index];
      
      // Set CoM target from the footstep plan using proper TSID API
      Eigen::Vector3d com_pos(
        current_step.com_position.x,
        current_step.com_position.y, 
        current_step.com_position.z);
      Eigen::Vector3d com_vel(
        current_step.com_velocity.x,
        current_step.com_velocity.y,
        current_step.com_velocity.z);
      Eigen::Vector3d com_acc = Eigen::Vector3d::Zero();  // For now
      
      tsid_controller_->setCoMReference(com_pos, com_vel, com_acc);
      
      // Set foot contact constraints based on support foot
      if (current_step.is_support_foot) {
        std::string foot_frame = current_step.is_right_foot ? "feet_2" : "feet";
        
        // Convert pose to transformation matrix
        Eigen::Matrix4d H_foot = Eigen::Matrix4d::Identity();
        H_foot(0,3) = current_step.pose.position.x;
        H_foot(1,3) = current_step.pose.position.y;
        H_foot(2,3) = current_step.pose.position.z;
        
        // Convert quaternion to rotation matrix
        Eigen::Quaterniond q(current_step.pose.orientation.w,
                           current_step.pose.orientation.x,
                           current_step.pose.orientation.y,
                           current_step.pose.orientation.z);
        H_foot.block<3,3>(0,0) = q.toRotationMatrix();
        
        tsid_controller_->setFootReference(foot_frame, H_foot);
        tsid_controller_->addFootContact(foot_frame);
        
        // Set swing foot if there's a next step
        if (msg->current_step_index + 1 < msg->footsteps.size()) {
          const auto& next_step = msg->footsteps[msg->current_step_index + 1];
          std::string swing_foot_frame = next_step.is_right_foot ? "feet_2" : "feet";
          
          Eigen::Matrix4d H_swing = Eigen::Matrix4d::Identity();
          H_swing(0,3) = next_step.pose.position.x;
          H_swing(1,3) = next_step.pose.position.y;
          H_swing(2,3) = next_step.pose.position.z;
          
          Eigen::Quaterniond q_swing(next_step.pose.orientation.w,
                                   next_step.pose.orientation.x,
                                   next_step.pose.orientation.y,
                                   next_step.pose.orientation.z);
          H_swing.block<3,3>(0,0) = q_swing.toRotationMatrix();
          
          tsid_controller_->setFootReference(swing_foot_frame, H_swing);
          tsid_controller_->removeFootContact(swing_foot_frame);
        }
      }
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Processing step %d/%zu: %s foot support", 
                          msg->current_step_index, msg->footsteps.size(),
                          current_step.is_right_foot ? "right" : "left");
    }
  }

  void setFootTarget(const std::string& foot_name, const geometry_msgs::msg::Pose& pose, bool in_contact)
  {
    robosoccer_control::TaskSpaceTarget foot_target;
    foot_target.position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    foot_target.orientation = Eigen::Quaterniond(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    foot_target.linear_velocity = Eigen::Vector3d::Zero();
    foot_target.angular_velocity = Eigen::Vector3d::Zero();
    foot_target.linear_acceleration = Eigen::Vector3d::Zero();
    foot_target.angular_acceleration = Eigen::Vector3d::Zero();
    foot_target.weight = in_contact ? 100.0 : 50.0;  // Higher weight when in contact
    foot_target.active = true;

    tsid_controller_->setEndEffectorTarget(foot_name, foot_target);

    // Update contact constraint
    if (in_contact) {
      robosoccer_control::ContactConstraint contact;
      contact.frame_name = foot_name;
      contact.is_active = true;
      contact.contact_normal = Eigen::Vector3d(0, 0, 1);  // Assume flat ground
      contact.friction_coefficient = 0.7;
      
      // Set reasonable force bounds for humanoid robot
      contact.wrench_bounds_min << -100, -100, 0, -10, -10, -10;  // [fx, fy, fz, mx, my, mz]
      contact.wrench_bounds_max << 100, 100, 1000, 10, 10, 10;
      
      tsid_controller_->setContactConstraint(foot_name, contact);
    } else {
      tsid_controller_->removeContactConstraint(foot_name);
    }
  }

  void controlLoop()
  {
    if (current_joint_positions_.empty()) {
      return;  // Wait for joint states
    }

    // Compute control torques using TSID
    Eigen::VectorXd tau_computed;
    if (tsid_controller_->computeControlTorques(tau_computed)) {
      
      // Publish computed torques (for analysis/feedforward)
      std_msgs::msg::Float64MultiArray torque_msg;
      torque_msg.data.resize(tau_computed.size());
      for (int i = 0; i < tau_computed.size(); ++i) {
        torque_msg.data[i] = tau_computed[i];
      }
      torque_commands_pub_->publish(torque_msg);

      // For now, publish position commands (integration would be needed for full torque control)
      // This is a simplified approach - in practice you'd want to integrate accelerations
      publishPositionCommands();
      
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Failed to compute TSID control torques");
    }
  }

  void publishPositionCommands()
  {
    // Simple approach: maintain current positions with small corrections
    // In practice, you'd integrate the computed accelerations to get position commands
    std_msgs::msg::Float64MultiArray cmd_msg;
    cmd_msg.data.resize(joint_names_.size());
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      // For now, just command current positions (this keeps robot stable)
      cmd_msg.data[i] = current_joint_positions_[i];
    }
    
    joint_commands_pub_->publish(cmd_msg);
  }

  void initializeDefaultContacts()
  {
    // Set both feet in contact by default (standing pose)
    // Using actual frame names from the URDF
    robosoccer_control::ContactConstraint left_contact, right_contact;
    
    left_contact.frame_name = "feet";  // Left foot in URDF
    left_contact.is_active = true;
    left_contact.contact_normal = Eigen::Vector3d(0, 0, 1);
    left_contact.friction_coefficient = 0.7;
    left_contact.wrench_bounds_min << -100, -100, 0, -10, -10, -10;
    left_contact.wrench_bounds_max << 100, 100, 1000, 10, 10, 10;

    right_contact = left_contact;
    right_contact.frame_name = "feet_2";  // Right foot in URDF

    tsid_controller_->setContactConstraint("feet", left_contact);
    tsid_controller_->setContactConstraint("feet_2", right_contact);

    RCLCPP_INFO(this->get_logger(), "Initialized default contact constraints (both feet: 'feet' and 'feet_2')");
  }

  // Member variables
  std::unique_ptr<robosoccer_control::TSIDController> tsid_controller_;
  std::vector<std::string> joint_names_;
  double control_frequency_;
  bool posture_target_set_ = false;

  // Current robot state
  std::vector<double> current_joint_positions_;
  std::vector<double> current_joint_velocities_;

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_commands_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_commands_pub_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<robosoccer_control::msg::TSIDCommand>::SharedPtr tsid_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr com_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_foot_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_foot_target_sub_;
  rclcpp::Subscription<robosoccer_control::msg::FootstepArray>::SharedPtr footstep_plan_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Motion planning state
  robosoccer_control::msg::FootstepArray::SharedPtr current_motion_plan_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TSIDControllerNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting TSID Controller Node...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
} 