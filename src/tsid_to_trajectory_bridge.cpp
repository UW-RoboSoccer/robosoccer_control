#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class TSIDToTrajectoryBridge : public rclcpp::Node
{
public:
  TSIDToTrajectoryBridge() : Node("tsid_to_trajectory_bridge")
  {
    // Joint names in the order expected by position_controller
    joint_names_ = {
      "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
      "left_shoulder_pitch", "left_shoulder_roll", "left_elbow", 
      "head_yaw", "head_pitch",
      "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch",
      "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch"
    };

    // TSID joint order (from our config)
    tsid_joint_names_ = {
      "right_elbow", "right_shoulder_roll", "right_shoulder_pitch",
      "left_elbow", "left_shoulder_roll", "left_shoulder_pitch", 
      "head_pitch", "head_yaw",
      "right_ankle_pitch", "right_knee", "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
      "left_ankle_pitch", "left_knee", "left_hip_yaw", "left_hip_roll", "left_hip_pitch"
    };

    // Create mapping from TSID order to controller order
    createJointMapping();

    // Publishers and subscribers
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/position_controller/joint_trajectory", 10);

    tsid_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/robosoccer/joint_commands", 10,
      std::bind(&TSIDToTrajectoryBridge::tsidCommandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TSID to Trajectory Bridge initialized!");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /robosoccer/joint_commands");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /position_controller/joint_trajectory");
  }

private:
  void createJointMapping()
  {
    joint_mapping_.resize(joint_names_.size());
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      // Find index of joint_names_[i] in tsid_joint_names_
      auto it = std::find(tsid_joint_names_.begin(), tsid_joint_names_.end(), joint_names_[i]);
      if (it != tsid_joint_names_.end()) {
        joint_mapping_[i] = std::distance(tsid_joint_names_.begin(), it);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Joint %s not found in TSID joint names!", joint_names_[i].c_str());
        joint_mapping_[i] = i; // fallback
      }
    }

    RCLCPP_INFO(this->get_logger(), "Joint mapping created for %zu joints", joint_names_.size());
  }

  void tsidCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != tsid_joint_names_.size()) {
      RCLCPP_WARN(this->get_logger(), "Received %zu joint commands, expected %zu", 
                  msg->data.size(), tsid_joint_names_.size());
      return;
    }

    // Create JointTrajectory message
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = this->now();
    traj_msg.header.frame_id = "";
    traj_msg.joint_names = joint_names_;

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(joint_names_.size());
    point.velocities.clear(); // Let controller handle velocities
    point.accelerations.clear();
    point.effort.clear();
    point.time_from_start = rclcpp::Duration::from_nanoseconds(100000000); // 0.1 seconds

    // Map joint positions from TSID order to controller order
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      point.positions[i] = msg->data[joint_mapping_[i]];
    }

    traj_msg.points.push_back(point);

    // Publish trajectory
    trajectory_pub_->publish(traj_msg);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tsid_sub_;
  
  std::vector<std::string> joint_names_;        // Expected by controller
  std::vector<std::string> tsid_joint_names_;   // From TSID config
  std::vector<size_t> joint_mapping_;           // Maps controller index to TSID index
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TSIDToTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
} 