#include <chrono>
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "drake/lcmt_panda_status.hpp"
#include "drake/lcmt_drake_signal.hpp"

namespace franka_driver {
namespace {

/**
 * @brief ROS 2 node that subscribes to multiple LCM channels and republishes to ROS topics.
 * 
 * This bridge enables full franka::RobotState access in ROS 2 ecosystem.
 * It subscribes to:
 *   - PANDA_STATUS (joint states, torques, control status)
 *   - PANDA_CARTESIAN (poses, velocities, wrenches)
 *   - PANDA_DYNAMICS (motor states, elbow, masses, inertias)
 *   - PANDA_CONTACT (collision/contact detection)
 * 
 * And publishes ~20 ROS topics covering all franka::RobotState fields.
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
    
    // Subscribe to LCM channels
    lcm_->subscribe(lcm_channel_, &LcmRos2Bridge::HandlePandaStatus, this);
    lcm_->subscribe("PANDA_CARTESIAN", &LcmRos2Bridge::HandleCartesian, this);
    lcm_->subscribe("PANDA_DYNAMICS", &LcmRos2Bridge::HandleDynamics, this);
    lcm_->subscribe("PANDA_CONTACT", &LcmRos2Bridge::HandleContact, this);

    // Subscribe to the SDF node
    // lcm_->subscribe("/nvblox/esdf_results", &LcmRos2Bridge::HandleSDF, this);
    
    // Create ROS publishers - Joint States
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/joint_states", 10);
    
    joint_state_desired_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/joint_states_desired", 10);
    
    external_torques_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/external_torques", 10);
    
    robot_status_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/franka/robot_status", 10);
    
    // Create ROS publishers - Cartesian States
    ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/franka/ee_pose", 10);
    ee_pose_desired_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/franka/ee_pose_desired", 10);
    ee_pose_commanded_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/franka/ee_pose_commanded", 10);
    
    ee_twist_desired_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/franka/ee_twist_desired", 10);
    ee_twist_commanded_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/franka/ee_twist_commanded", 10);
    ee_accel_commanded_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
        "/franka/ee_accel_commanded", 10);
    
    ee_wrench_base_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/franka/ee_wrench_base", 10);
    ee_wrench_stiffness_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/franka/ee_wrench_stiffness", 10);
    
    // Create ROS publishers - Dynamics
    motor_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/motor_states", 10);
    torque_derivatives_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/torque_derivatives", 10);
    elbow_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/franka/elbow_state", 10);
    mass_properties_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/franka/mass_properties", 10);
    
    // Create ROS publishers - Contact/Collision
    joint_contact_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/joint_contact", 10);
    joint_collision_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/franka/joint_collision", 10);
    cartesian_contact_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/franka/cartesian_contact", 10);
    cartesian_collision_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/franka/cartesian_collision", 10);
    
    // Set up joint names for Franka Panda
    joint_names_ = {
      "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"
    };
    
    // Create timer to handle LCM messages
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
        1ms, std::bind(&LcmRos2Bridge::HandleLcm, this));
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "LCM to ROS 2 Bridge - FULL ROBOT STATE");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "LCM URL: %s", 
                lcm_url_.empty() ? "default (udpm://239.255.76.67:7667)" : lcm_url_.c_str());
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "Subscribing to LCM channels:");
    RCLCPP_INFO(this->get_logger(), "  - PANDA_STATUS");
    RCLCPP_INFO(this->get_logger(), "  - PANDA_CARTESIAN");
    RCLCPP_INFO(this->get_logger(), "  - PANDA_DYNAMICS");
    RCLCPP_INFO(this->get_logger(), "  - PANDA_CONTACT");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "Publishing ~20 ROS topics:");
    RCLCPP_INFO(this->get_logger(), "  Joint States:");
    RCLCPP_INFO(this->get_logger(), "    /franka/joint_states (measured q, dq, tau_J)");
    RCLCPP_INFO(this->get_logger(), "    /franka/joint_states_desired");
    RCLCPP_INFO(this->get_logger(), "    /franka/external_torques");
    RCLCPP_INFO(this->get_logger(), "  Cartesian:");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_pose (O_T_EE)");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_pose_desired (O_T_EE_d)");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_pose_commanded (O_T_EE_c)");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_twist_desired");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_twist_commanded");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_accel_commanded");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_wrench_base (O_F_ext_hat_K)");
    RCLCPP_INFO(this->get_logger(), "    /franka/ee_wrench_stiffness (K_F_ext_hat_K)");
    RCLCPP_INFO(this->get_logger(), "  Dynamics:");
    RCLCPP_INFO(this->get_logger(), "    /franka/motor_states (theta, dtheta)");
    RCLCPP_INFO(this->get_logger(), "    /franka/torque_derivatives (dtau_J)");
    RCLCPP_INFO(this->get_logger(), "    /franka/elbow_state");
    RCLCPP_INFO(this->get_logger(), "    /franka/mass_properties");
    RCLCPP_INFO(this->get_logger(), "  Contact/Collision:");
    RCLCPP_INFO(this->get_logger(), "    /franka/joint_contact");
    RCLCPP_INFO(this->get_logger(), "    /franka/joint_collision");
    RCLCPP_INFO(this->get_logger(), "    /franka/cartesian_contact");
    RCLCPP_INFO(this->get_logger(), "    /franka/cartesian_collision");
    RCLCPP_INFO(this->get_logger(), "  Status:");
    RCLCPP_INFO(this->get_logger(), "    /franka/robot_status");
    RCLCPP_INFO(this->get_logger(), "========================================");
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
    
    // Common header
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "panda_link0";
    
    const size_t num_joints = std::min(
        static_cast<size_t>(msg->num_joints), 
        joint_names_.size());
    
    // =========================================================================
    // 1. Measured Joint States
    // =========================================================================
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header = header;
    joint_state.name.assign(joint_names_.begin(), 
                           joint_names_.begin() + num_joints);
    joint_state.position.assign(msg->joint_position.begin(),
                               msg->joint_position.begin() + num_joints);
    joint_state.velocity.assign(msg->joint_velocity.begin(),
                               msg->joint_velocity.begin() + num_joints);
    joint_state.effort.assign(msg->joint_torque.begin(),
                             msg->joint_torque.begin() + num_joints);
    joint_state_pub_->publish(joint_state);
    
    // =========================================================================
    // 2. Desired Joint States
    // =========================================================================
    auto joint_state_desired = sensor_msgs::msg::JointState();
    joint_state_desired.header = header;
    joint_state_desired.name.assign(joint_names_.begin(), 
                                   joint_names_.begin() + num_joints);
    joint_state_desired.position.assign(msg->joint_position_desired.begin(),
                                       msg->joint_position_desired.begin() + num_joints);
    joint_state_desired.velocity.assign(msg->joint_velocity_desired.begin(),
                                       msg->joint_velocity_desired.begin() + num_joints);
    joint_state_desired.effort.assign(msg->joint_torque_desired.begin(),
                                     msg->joint_torque_desired.begin() + num_joints);
    joint_state_desired_pub_->publish(joint_state_desired);
    
    // =========================================================================
    // 3. External Torques
    // =========================================================================
    auto external_torques = sensor_msgs::msg::JointState();
    external_torques.header = header;
    external_torques.name.assign(joint_names_.begin(), 
                                joint_names_.begin() + num_joints);
    // Position and velocity not applicable for external torques
    external_torques.effort.assign(msg->joint_torque_external.begin(),
                                  msg->joint_torque_external.begin() + num_joints);
    external_torques_pub_->publish(external_torques);
    
    // =========================================================================
    // 4. Robot Status (control success rate, robot mode, control mode)
    // =========================================================================
    auto robot_status = std_msgs::msg::Float64MultiArray();
    robot_status.data.push_back(msg->control_command_success_rate);
    robot_status.data.push_back(static_cast<double>(msg->robot_mode));
    robot_status.data.push_back(static_cast<double>(msg->control_mode));
    robot_status_pub_->publish(robot_status);
    
    message_count_++;
    if (message_count_ % 100 == 0) {
      RCLCPP_DEBUG(this->get_logger(), "Published %ld messages", message_count_);
    }
  }
  
  void HandleCartesian(const lcm::ReceiveBuffer* rbuf,
                      const std::string& channel,
                      const drake::lcmt_drake_signal* msg) {
    (void)rbuf; (void)channel;
    
    auto header = std_msgs::msg::Header();
    header.stamp = this->now();
    header.frame_id = "panda_link0";
    
    // Helper to extract 4x4 matrix starting at offset
    auto extract_pose = [&](int offset) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      // Convert 4x4 column-major matrix to position + quaternion
      // Position: last column (elements 12, 13, 14)
      pose.pose.position.x = msg->val[offset + 12];
      pose.pose.position.y = msg->val[offset + 13];
      pose.pose.position.z = msg->val[offset + 14];
      // Rotation matrix to quaternion (simplified - use identity for now)
      // TODO: Implement proper rotation matrix to quaternion conversion
      pose.pose.orientation.w = 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      return pose;
    };
    
    // Publish poses
    ee_pose_pub_->publish(extract_pose(0));           // O_T_EE
    ee_pose_desired_pub_->publish(extract_pose(16));  // O_T_EE_d
    ee_pose_commanded_pub_->publish(extract_pose(32)); // O_T_EE_c
    
    // Publish twists and accelerations
    auto twist_desired = geometry_msgs::msg::TwistStamped();
    twist_desired.header = header;
    int twist_offset = 80; // After 5 * 16 pose elements
    twist_desired.twist.linear.x = msg->val[twist_offset];
    twist_desired.twist.linear.y = msg->val[twist_offset + 1];
    twist_desired.twist.linear.z = msg->val[twist_offset + 2];
    twist_desired.twist.angular.x = msg->val[twist_offset + 3];
    twist_desired.twist.angular.y = msg->val[twist_offset + 4];
    twist_desired.twist.angular.z = msg->val[twist_offset + 5];
    ee_twist_desired_pub_->publish(twist_desired);
    
    auto twist_commanded = geometry_msgs::msg::TwistStamped();
    twist_commanded.header = header;
    twist_commanded.twist.linear.x = msg->val[twist_offset + 6];
    twist_commanded.twist.linear.y = msg->val[twist_offset + 7];
    twist_commanded.twist.linear.z = msg->val[twist_offset + 8];
    twist_commanded.twist.angular.x = msg->val[twist_offset + 9];
    twist_commanded.twist.angular.y = msg->val[twist_offset + 10];
    twist_commanded.twist.angular.z = msg->val[twist_offset + 11];
    ee_twist_commanded_pub_->publish(twist_commanded);
    
    auto accel_commanded = geometry_msgs::msg::AccelStamped();
    accel_commanded.header = header;
    accel_commanded.accel.linear.x = msg->val[twist_offset + 12];
    accel_commanded.accel.linear.y = msg->val[twist_offset + 13];
    accel_commanded.accel.linear.z = msg->val[twist_offset + 14];
    accel_commanded.accel.angular.x = msg->val[twist_offset + 15];
    accel_commanded.accel.angular.y = msg->val[twist_offset + 16];
    accel_commanded.accel.angular.z = msg->val[twist_offset + 17];
    ee_accel_commanded_pub_->publish(accel_commanded);
    
    // Publish wrenches
    auto wrench_base = geometry_msgs::msg::WrenchStamped();
    wrench_base.header = header;
    int wrench_offset = 98; // After poses + twists
    wrench_base.wrench.force.x = msg->val[wrench_offset];
    wrench_base.wrench.force.y = msg->val[wrench_offset + 1];
    wrench_base.wrench.force.z = msg->val[wrench_offset + 2];
    wrench_base.wrench.torque.x = msg->val[wrench_offset + 3];
    wrench_base.wrench.torque.y = msg->val[wrench_offset + 4];
    wrench_base.wrench.torque.z = msg->val[wrench_offset + 5];
    ee_wrench_base_pub_->publish(wrench_base);
    
    auto wrench_stiffness = geometry_msgs::msg::WrenchStamped();
    wrench_stiffness.header = header;
    wrench_stiffness.header.frame_id = "panda_K";
    wrench_stiffness.wrench.force.x = msg->val[wrench_offset + 6];
    wrench_stiffness.wrench.force.y = msg->val[wrench_offset + 7];
    wrench_stiffness.wrench.force.z = msg->val[wrench_offset + 8];
    wrench_stiffness.wrench.torque.x = msg->val[wrench_offset + 9];
    wrench_stiffness.wrench.torque.y = msg->val[wrench_offset + 10];
    wrench_stiffness.wrench.torque.z = msg->val[wrench_offset + 11];
    ee_wrench_stiffness_pub_->publish(wrench_stiffness);
  }
  
  void HandleDynamics(const lcm::ReceiveBuffer* rbuf,
                     const std::string& channel,
                     const drake::lcmt_drake_signal* msg) {
    (void)rbuf; (void)channel;
    
    auto header = std_msgs::msg::Header();
    header.stamp = this->now();
    header.frame_id = "panda_link0";
    
    // Motor states (theta, dtheta)
    auto motor_states = sensor_msgs::msg::JointState();
    motor_states.header = header;
    motor_states.name = joint_names_;
    motor_states.position.resize(7);
    motor_states.velocity.resize(7);
    for (int i = 0; i < 7; ++i) {
      motor_states.position[i] = msg->val[i];      // theta
      motor_states.velocity[i] = msg->val[7 + i];   // dtheta
    }
    motor_states_pub_->publish(motor_states);
    
    // Torque derivatives
    auto torque_deriv = sensor_msgs::msg::JointState();
    torque_deriv.header = header;
    torque_deriv.name = joint_names_;
    torque_deriv.effort.resize(7);
    for (int i = 0; i < 7; ++i) {
      torque_deriv.effort[i] = msg->val[14 + i];  // dtau_J
    }
    torque_derivatives_pub_->publish(torque_deriv);
    
    // Elbow state (elbow, elbow_d, elbow_c, delbow_c, ddelbow_c)
    auto elbow_state = std_msgs::msg::Float64MultiArray();
    elbow_state.data.resize(10);
    for (int i = 0; i < 10; ++i) {
      elbow_state.data[i] = msg->val[21 + i];
    }
    elbow_state_pub_->publish(elbow_state);
    
    // Mass properties (F_x_Cee, m_ee, F_x_Cload, m_load, F_x_Ctotal, m_total, I_ee, I_load, I_total)
    auto mass_props = std_msgs::msg::Float64MultiArray();
    mass_props.data.resize(msg->dim - 31);  // Remaining elements
    for (size_t i = 31; i < msg->val.size(); ++i) {
      mass_props.data[i - 31] = msg->val[i];
    }
    mass_properties_pub_->publish(mass_props);
  }

//   void HandleSDF(const lcm::ReceiveBuffer* rbuf,
//                const std::string& channel,
//                const lcm::String* msg) {
//   (void)rbuf;  // Unused
//   (void)channel;  // Unused

//   // Print the received JSON string message
//   RCLCPP_INFO(this->get_logger(), "Received message on channel: %s", channel.c_str());
//   RCLCPP_INFO(this->get_logger(), "Message: %s", msg->data.c_str());
// }
  
  void HandleContact(const lcm::ReceiveBuffer* rbuf,
                    const std::string& channel,
                    const drake::lcmt_drake_signal* msg) {
    (void)rbuf; (void)channel;
    
    auto header = std_msgs::msg::Header();
    header.stamp = this->now();
    header.frame_id = "panda_link0";
    
    // Joint contact
    auto joint_contact = sensor_msgs::msg::JointState();
    joint_contact.header = header;
    joint_contact.name = joint_names_;
    joint_contact.effort.resize(7);
    for (int i = 0; i < 7; ++i) {
      joint_contact.effort[i] = msg->val[i];
    }
    joint_contact_pub_->publish(joint_contact);
    
    // Joint collision
    auto joint_collision = sensor_msgs::msg::JointState();
    joint_collision.header = header;
    joint_collision.name = joint_names_;
    joint_collision.effort.resize(7);
    for (int i = 0; i < 7; ++i) {
      joint_collision.effort[i] = msg->val[7 + i];
    }
    joint_collision_pub_->publish(joint_collision);
    
    // Cartesian contact
    auto cart_contact = std_msgs::msg::Float64MultiArray();
    cart_contact.data.resize(6);
    for (int i = 0; i < 6; ++i) {
      cart_contact.data[i] = msg->val[14 + i];
    }
    cartesian_contact_pub_->publish(cart_contact);
    
    // Cartesian collision
    auto cart_collision = std_msgs::msg::Float64MultiArray();
    cart_collision.data.resize(6);
    for (int i = 0; i < 6; ++i) {
      cart_collision.data[i] = msg->val[20 + i];
    }
    cartesian_collision_pub_->publish(cart_collision);
  }
  
  std::unique_ptr<lcm::LCM> lcm_;
  std::string lcm_url_;
  std::string lcm_channel_;
  std::vector<std::string> joint_names_;
  
  // Joint state publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_desired_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr external_torques_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr robot_status_pub_;
  
  // Cartesian publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_desired_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_commanded_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_twist_desired_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_twist_commanded_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr ee_accel_commanded_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ee_wrench_base_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ee_wrench_stiffness_pub_;
  
  // Dynamics publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_derivatives_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr elbow_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mass_properties_pub_;
  
  // Contact/Collision publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_contact_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_collision_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_collision_pub_;
  
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

