#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class SimpleBridgeTest : public rclcpp::Node
{
public:
  SimpleBridgeTest() : Node("simple_bridge_test")
  {
    // Joint names matching your robot
    joint_names_ = {
      "right_elbow", "right_shoulder_roll", "right_shoulder_pitch",
      "left_elbow", "left_shoulder_roll", "left_shoulder_pitch", 
      "head_pitch", "head_yaw",
      "right_ankle_pitch", "right_knee", "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
      "left_ankle_pitch", "left_knee", "left_hip_yaw", "left_hip_roll", "left_hip_pitch"
    };

    // Publisher
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/position_controller/joint_trajectory", 10);

    // Timer to send test commands
    auto timer_period = std::chrono::seconds(1);
    test_timer_ = this->create_wall_timer(
      timer_period, std::bind(&SimpleBridgeTest::sendTestCommand, this));

    RCLCPP_INFO(this->get_logger(), "Simple Bridge Test initialized!");
    RCLCPP_INFO(this->get_logger(), "Will send test trajectory every second");
    RCLCPP_INFO(this->get_logger(), "Joint count: %zu", joint_names_.size());
  }

private:
  void sendTestCommand()
  {
    // Create a simple trajectory
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    traj_msg.header.stamp = this->now();
    traj_msg.joint_names = joint_names_;

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions.resize(joint_names_.size(), 0.0);  // All joints at 0
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);

    traj_msg.points.push_back(point);
    trajectory_pub_->publish(traj_msg);

    RCLCPP_INFO(this->get_logger(), "Sent test trajectory with %zu joints", joint_names_.size());
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr test_timer_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBridgeTest>());
  rclcpp::shutdown();
  return 0;
} 