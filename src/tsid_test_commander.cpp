#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robosoccer_control/msg/tsid_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TSIDTestCommander : public rclcpp::Node
{
public:
  TSIDTestCommander() : Node("tsid_test_commander")
  {
    // Declare parameters
    this->declare_parameter("test_mode", "standing");  // standing, com_tracking, foot_tracking
    this->declare_parameter("use_sim_time", false);
    this->declare_parameter("command_frequency", 10.0);  // Hz
    
    // Get parameters
    test_mode_ = this->get_parameter("test_mode").as_string();
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
    command_frequency_ = this->get_parameter("command_frequency").as_double();

    // Publishers
    tsid_command_pub_ = this->create_publisher<robosoccer_control::msg::TSIDCommand>(
      "/tsid_commands", 10);
    
    com_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/com_target", 10);
    
    left_foot_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/left_foot_target", 10);
    
    right_foot_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/right_foot_target", 10);

    // Control timer
    auto timer_period = std::chrono::duration<double>(1.0 / command_frequency_);
    command_timer_ = this->create_wall_timer(
      timer_period, std::bind(&TSIDTestCommander::commandLoop, this));

    // Initialize test state
    test_time_ = 0.0;
    phase_ = 0;

    RCLCPP_INFO(this->get_logger(), "TSID Test Commander initialized!");
    RCLCPP_INFO(this->get_logger(), "Test mode: %s", test_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "Command frequency: %.1f Hz", command_frequency_);
    RCLCPP_INFO(this->get_logger(), "Publishing to: /tsid_commands, /com_target, /left_foot_target, /right_foot_target");
  }

private:
  void commandLoop()
  {
    test_time_ += 1.0 / command_frequency_;
    
    if (test_mode_ == "standing") {
      sendStandingCommand();
    } else if (test_mode_ == "com_tracking") {
      sendCoMTrackingCommand();
    } else if (test_mode_ == "foot_tracking") {
      sendFootTrackingCommand();
    } else if (test_mode_ == "simple_motion") {
      sendSimpleMotionCommand();
    }
  }

  void sendStandingCommand()
  {
    // Send a standing command (CoM at center, feet in place)
    auto tsid_cmd = robosoccer_control::msg::TSIDCommand();
    tsid_cmd.header.stamp = this->now();
    
    // CoM at standing height
    tsid_cmd.desired_com_pose.position.x = 0.0;
    tsid_cmd.desired_com_pose.position.y = 0.0;
    tsid_cmd.desired_com_pose.position.z = 0.4;  // Standing height
    tsid_cmd.desired_com_pose.orientation.w = 1.0;
    
    // Zero velocity
    tsid_cmd.desired_com_velocity.linear.x = 0.0;
    tsid_cmd.desired_com_velocity.linear.y = 0.0;
    tsid_cmd.desired_com_velocity.linear.z = 0.0;
    
    // Feet in contact
    tsid_cmd.left_foot_contact = true;
    tsid_cmd.right_foot_contact = true;
    
    // Publish command
    tsid_command_pub_->publish(tsid_cmd);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Sending standing command - CoM at (0, 0, 0.4)");
  }

  void sendCoMTrackingCommand()
  {
    // Send a CoM tracking command (circular motion)
    auto tsid_cmd = robosoccer_control::msg::TSIDCommand();
    tsid_cmd.header.stamp = this->now();
    
    // Circular CoM motion
    double radius = 0.05;  // 5cm radius
    double frequency = 0.5;  // 0.5 Hz
    
    tsid_cmd.desired_com_pose.position.x = radius * cos(2 * M_PI * frequency * test_time_);
    tsid_cmd.desired_com_pose.position.y = radius * sin(2 * M_PI * frequency * test_time_);
    tsid_cmd.desired_com_pose.position.z = 0.4;
    tsid_cmd.desired_com_pose.orientation.w = 1.0;
    
    // CoM velocity
    tsid_cmd.desired_com_velocity.linear.x = -radius * 2 * M_PI * frequency * sin(2 * M_PI * frequency * test_time_);
    tsid_cmd.desired_com_velocity.linear.y = radius * 2 * M_PI * frequency * cos(2 * M_PI * frequency * test_time_);
    tsid_cmd.desired_com_velocity.linear.z = 0.0;
    
    // Feet in contact
    tsid_cmd.left_foot_contact = true;
    tsid_cmd.right_foot_contact = true;
    
    // Publish command
    tsid_command_pub_->publish(tsid_cmd);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Sending CoM tracking command - Circular motion");
  }

  void sendFootTrackingCommand()
  {
    // Send foot tracking commands
    auto left_foot_msg = geometry_msgs::msg::PoseStamped();
    auto right_foot_msg = geometry_msgs::msg::PoseStamped();
    
    left_foot_msg.header.stamp = this->now();
    right_foot_msg.header.stamp = this->now();
    
    // Simple foot motion (lifting one foot)
    if (phase_ < 100) {
      // Phase 1: Lift right foot
      right_foot_msg.pose.position.z = 0.05 * sin(M_PI * test_time_);
      left_foot_msg.pose.position.z = 0.0;
    } else if (phase_ < 200) {
      // Phase 2: Lift left foot
      right_foot_msg.pose.position.z = 0.0;
      left_foot_msg.pose.position.z = 0.05 * sin(M_PI * test_time_);
    } else {
      // Reset phase
      phase_ = 0;
    }
    
    // Publish foot targets
    left_foot_target_pub_->publish(left_foot_msg);
    right_foot_target_pub_->publish(right_foot_msg);
    
    phase_++;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Sending foot tracking command - Phase %d", phase_);
  }

  void sendSimpleMotionCommand()
  {
    // Send a simple motion command (CoM forward/backward)
    auto tsid_cmd = robosoccer_control::msg::TSIDCommand();
    tsid_cmd.header.stamp = this->now();
    
    // Forward/backward motion
    double amplitude = 0.02;  // 2cm
    double frequency = 0.3;   // 0.3 Hz
    
    tsid_cmd.desired_com_pose.position.x = amplitude * sin(2 * M_PI * frequency * test_time_);
    tsid_cmd.desired_com_pose.position.y = 0.0;
    tsid_cmd.desired_com_pose.position.z = 0.4;
    tsid_cmd.desired_com_pose.orientation.w = 1.0;
    
    // CoM velocity
    tsid_cmd.desired_com_velocity.linear.x = amplitude * 2 * M_PI * frequency * cos(2 * M_PI * frequency * test_time_);
    tsid_cmd.desired_com_velocity.linear.y = 0.0;
    tsid_cmd.desired_com_velocity.linear.z = 0.0;
    
    // Feet in contact
    tsid_cmd.left_foot_contact = true;
    tsid_cmd.right_foot_contact = true;
    
    // Publish command
    tsid_command_pub_->publish(tsid_cmd);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Sending simple motion command - Forward/backward");
  }

  rclcpp::Publisher<robosoccer_control::msg::TSIDCommand>::SharedPtr tsid_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr com_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_foot_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_foot_target_pub_;
  
  rclcpp::TimerBase::SharedPtr command_timer_;
  
  std::string test_mode_;
  bool use_sim_time_;
  double command_frequency_;
  double test_time_;
  int phase_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TSIDTestCommander>());
  rclcpp::shutdown();
  return 0;
} 