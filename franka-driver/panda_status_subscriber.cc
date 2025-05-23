#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_panda_status.hpp"

class PandaStatusHandler {
public:
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
                      const std::string& chan,
                      const drake::lcmt_panda_status* msg) {
        std::cout << "\n=== Received PANDA_STATUS message ===" << std::endl;
        std::cout << "Timestamp: " << msg->utime << std::endl;
        std::cout << "Robot timestamp: " << msg->robot_utime << std::endl;
        std::cout << "Number of joints: " << msg->num_joints << std::endl;
        
        // Print joint positions
        std::cout << "Joint positions: [";
        for (size_t i = 0; i < msg->joint_position.size(); ++i) {
            std::cout << msg->joint_position[i];
            if (i < msg->joint_position.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Print joint velocities
        std::cout << "Joint velocities: [";
        for (size_t i = 0; i < msg->joint_velocity.size(); ++i) {
            std::cout << msg->joint_velocity[i];
            if (i < msg->joint_velocity.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Print joint torques
        std::cout << "Joint torques: [";
        for (size_t i = 0; i < msg->joint_torque.size(); ++i) {
            std::cout << msg->joint_torque[i];
            if (i < msg->joint_torque.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // Print robot mode
        std::cout << "Robot mode: " << static_cast<int>(msg->robot_mode) << std::endl;
        std::cout << "Control success rate: " << msg->control_command_success_rate << std::endl;
        std::cout << "=== End of message ===\n" << std::endl;
    }
};

int main(int argc, char** argv) {
    // Create LCM instance
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Error: Failed to initialize LCM" << std::endl;
        return 1;
    }

    // Create handler
    PandaStatusHandler handler;

    // Subscribe to PANDA_STATUS channel
    lcm.subscribe("PANDA_STATUS", &PandaStatusHandler::handleMessage, &handler);

    std::cout << "Subscribing to PANDA_STATUS channel..." << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;

    // Handle messages
    while (0 == lcm.handle()) {
        // Keep running
    }

    return 0;
} 