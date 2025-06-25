#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TSIDToTrajectoryBridge : public rclcpp::Node
{
public:
  TSIDToTrajectoryBridge() : Node("tsid_to_trajectory_bridge")
  {
    // Declare parameters
    this->declare_parameter("trajectory_duration", 0.05);  // 50ms trajectory duration
    this->declare_parameter("max_velocity", 10.0);         // rad/s
    this->declare_parameter("max_acceleration", 50.0);     // rad/s^2
    this->declare_parameter("use_sim_time", false);
    
    // Get parameters
    trajectory_duration_ = this->get_parameter("trajectory_duration").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();

    // Joint names in EXACT order from both TSID config and simulation controller
    joint_names_ = {
      "right_elbow", "right_shoulder_roll", "right_shoulder_pitch",
      "left_elbow", "left_shoulder_roll", "left_shoulder_pitch", 
      "head_pitch", "head_yaw",
      "right_ankle_pitch", "right_knee", "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
      "left_ankle_pitch", "left_knee", "left_hip_yaw", "left_hip_roll", "left_hip_pitch"
    };

    // Initialize current joint state
    current_positions_.resize(joint_names_.size(), 0.0);
    current_velocities_.resize(joint_names_.size(), 0.0);
    last_command_time_ = this->now();

    // Joint orders now match exactly - no mapping needed!
    RCLCPP_INFO(this->get_logger(), "Joint orders match exactly - direct 1:1 mapping");
    RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.3f s", trajectory_duration_);

    // Publishers and subscribers
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/position_controller/joint_trajectory", 10);

    tsid_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/robosoccer/joint_commands", 10,
      std::bind(&TSIDToTrajectoryBridge::tsidCommandCallback, this, std::placeholders::_1));

    // Subscribe to joint states for velocity calculation
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&TSIDToTrajectoryBridge::jointStatesCallback, this, std::placeholders::_1));

    // Status publisher for monitoring
    status_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/bridge_status", 10);

    RCLCPP_INFO(this->get_logger(), "Enhanced TSID to Trajectory Bridge initialized!");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /robosoccer/joint_commands");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /joint_states");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /position_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "Publishing status to: /bridge_status");
  }

private:

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Update current joint state
    if (msg->position.size() >= joint_names_.size() && 
        msg->velocity.size() >= joint_names_.size()) {
      
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        current_positions_[i] = msg->position[i];
        current_velocities_[i] = msg->velocity[i];
      }
      
      // Update last joint state time
      last_joint_state_time_ = this->now();
    }
  }

  void tsidCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != joint_names_.size()) {
      RCLCPP_WARN(this->get_logger(), "Received %zu joint commands, expected %zu", 
                  msg->data.size(), joint_names_.size());
      return;
    }

    // Check if we have recent joint states
    auto now = this->now();
    if ((now - last_joint_state_time_).seconds() > 0.1) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "No recent joint states received. Waiting for joint state feedback.");
      return;
    }

    // Create JointTrajectory message
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = now;
    traj_msg.header.frame_id = "";
    traj_msg.joint_names = joint_names_;

    // Create trajectory point with enhanced features
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(joint_names_.size());
    point.velocities.resize(joint_names_.size());
    point.accelerations.resize(joint_names_.size());
    point.effort.clear();

    // Calculate time from start (use parameter)
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_duration_);

    // Enhanced mapping with velocity and acceleration calculation
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      double target_position = msg->data[i];
      double current_position = current_positions_[i];
      double current_velocity = current_velocities_[i];
      
      // Position mapping
      point.positions[i] = target_position;
      
      // Calculate desired velocity (simple proportional)
      double position_error = target_position - current_position;
      double desired_velocity = position_error / trajectory_duration_;
      
      // Limit velocity
      desired_velocity = std::max(-max_velocity_, std::min(max_velocity_, desired_velocity));
      point.velocities[i] = desired_velocity;
      
      // Calculate acceleration (simple derivative)
      double velocity_error = desired_velocity - current_velocity;
      double desired_acceleration = velocity_error / trajectory_duration_;
      
      // Limit acceleration
      desired_acceleration = std::max(-max_acceleration_, std::min(max_acceleration_, desired_acceleration));
      point.accelerations[i] = desired_acceleration;
    }

    traj_msg.points.push_back(point);

    // Publish trajectory
    trajectory_pub_->publish(traj_msg);
    
    // Update last command time
    last_command_time_ = now;
    
    // Publish status for monitoring
    publishStatus();
  }

  void publishStatus()
  {
    std_msgs::msg::Float64MultiArray status_msg;
    status_msg.data.resize(4);
    
    auto now = this->now();
    status_msg.data[0] = (now - last_command_time_).seconds();  // Time since last command
    status_msg.data[1] = (now - last_joint_state_time_).seconds();  // Time since last joint state
    status_msg.data[2] = trajectory_duration_;  // Current trajectory duration
    status_msg.data[3] = static_cast<double>(joint_names_.size());  // Number of joints
    
    status_pub_->publish(status_msg);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tsid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  
  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> current_velocities_;
  
  double trajectory_duration_;
  double max_velocity_;
  double max_acceleration_;
  bool use_sim_time_;
  
  rclcpp::Time last_command_time_;
  rclcpp::Time last_joint_state_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TSIDToTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
} 