#include <chrono>
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "drake/lcmt_panda_status.hpp"

namespace franka_driver {
namespace {

/**
 * @brief ROS 2 node that subscribes to LCM PANDA_STATUS and republishes as ROS JointState.
 * 
 * This bridge enables integration with ROS 2 ecosystem without modifying the franka_driver.
 * It subscribes to drake::lcmt_panda_status via LCM and publishes sensor_msgs::JointState.
 */
class LcmRos2Bridge : public rclcpp::Node {
 public:
  LcmRos2Bridge()
      : Node("lcm_ros2_bridge"),
        lcm_url_(""),
        lcm_channel_("PANDA_STATUS") {
    
    // Declare and get parameters
    this->declare_parameter("lcm_url", "");
    this->declare_parameter("lcm_channel", "PANDA_STATUS");
    this->declare_parameter("ros_topic", "/franka/joint_states");
    
    lcm_url_ = this->get_parameter("lcm_url").as_string();
    lcm_channel_ = this->get_parameter("lcm_channel").as_string();
    std::string ros_topic = this->get_parameter("ros_topic").as_string();
    
    // Initialize LCM
    if (lcm_url_.empty()) {
      lcm_ = std::make_unique<lcm::LCM>();
    } else {
      lcm_ = std::make_unique<lcm::LCM>(lcm_url_);
    }
    
    if (!lcm_->good()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize LCM with URL: %s", 
                   lcm_url_.empty() ? "default" : lcm_url_.c_str());
      throw std::runtime_error("LCM initialization failed");
    }
    
    // Subscribe to LCM channel
    lcm_->subscribe(lcm_channel_, &LcmRos2Bridge::HandlePandaStatus, this);
    
    // Create ROS publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        ros_topic, 10);
    
    // Set up joint names for Franka Panda
    joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    };
    
    // Create timer to handle LCM messages
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
        1ms, std::bind(&LcmRos2Bridge::HandleLcm, this));
    
    RCLCPP_INFO(this->get_logger(), "LCM to ROS 2 Bridge initialized");
    RCLCPP_INFO(this->get_logger(), "  LCM URL: %s", 
                lcm_url_.empty() ? "default (udpm://239.255.76.67:7667)" : lcm_url_.c_str());
    RCLCPP_INFO(this->get_logger(), "  LCM channel: %s", lcm_channel_.c_str());
    RCLCPP_INFO(this->get_logger(), "  ROS topic: %s", ros_topic.c_str());
  }
  
 private:
  void HandleLcm() {
    // Handle LCM with timeout (non-blocking)
    lcm_->handleTimeout(0);  // 0ms timeout = non-blocking
  }
  
  void HandlePandaStatus(const lcm::ReceiveBuffer* rbuf,
                         const std::string& channel,
                         const drake::lcmt_panda_status* msg) {
    (void)rbuf;    // Unused
    (void)channel; // Unused
    
    // Create ROS JointState message
    auto joint_state = sensor_msgs::msg::JointState();
    
    // Set header
    joint_state.header.stamp = this->now();
    joint_state.header.frame_id = "panda_link0";
    
    // Set joint names (use only the number of joints in the message)
    const size_t num_joints = std::min(
        static_cast<size_t>(msg->num_joints), 
        joint_names_.size());
    joint_state.name.assign(joint_names_.begin(), 
                           joint_names_.begin() + num_joints);
    
    // Set positions, velocities, and efforts (torques)
    joint_state.position.assign(msg->joint_position.begin(),
                               msg->joint_position.begin() + num_joints);
    joint_state.velocity.assign(msg->joint_velocity.begin(),
                               msg->joint_velocity.begin() + num_joints);
    joint_state.effort.assign(msg->joint_torque.begin(),
                             msg->joint_torque.begin() + num_joints);
    
    // Publish to ROS
    joint_state_pub_->publish(joint_state);
    
    message_count_++;
    if (message_count_ % 100 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "Published %ld messages", message_count_);
    }
  }
  
  std::unique_ptr<lcm::LCM> lcm_;
  std::string lcm_url_;
  std::string lcm_channel_;
  std::vector<std::string> joint_names_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  size_t message_count_{0};
};

}  // namespace
}  // namespace franka_driver

int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  try {
    // Create and spin the bridge node
    auto node = std::make_shared<franka_driver::LcmRos2Bridge>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("lcm_ros2_bridge"), 
                 "Exception in bridge: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}

