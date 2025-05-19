#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "biped_controller_utils.hpp"

using namespace std::chrono_literals;

class BipedControllerNode : public rclcpp::Node {
public:
  BipedControllerNode()
  : Node("biped_controller_node")
  {
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_cmd", 10); 

    footsteps_sub =
      this->create_subscription<std_msgs::msg::String>(
        "/footsteps", 10,
        std::bind(&BipedControllerNode::footsteps_callback, this,
              std::placeholders::_1));
    
    target_pose_sub =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10,
        std::bind(&BipedControllerNode::target_callback, this,
              std::placeholders::_1));
    
    // Setup Timer to publish at 50Hz (20ms)
    timer_ = this->create_wall_timer(
      20ms, std::bind(&BipedControllerNode::communication_loop, this));

    RCLCPP_INFO(this->get_logger(), "Biped controller node has been started.");
  }

private:
  void footsteps_callback(const std_msgs::msg::String::SharedPtr msg) {
  }

  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
   
  }

  void communication_loop() {
    // todo publish joint cmds?? or only publish in callbacks??
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr footsteps_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BipedControllerNode>());
  rclcpp::shutdown();
  return 0;
}