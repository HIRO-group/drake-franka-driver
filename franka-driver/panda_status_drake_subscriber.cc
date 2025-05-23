#include <memory>
#include <iostream>

#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace systems {

// A simple system that prints the received Panda status messages
class PandaStatusPrinter : public LeafSystem<double> {
 public:
  PandaStatusPrinter() {
    // Declare an abstract input port that receives Panda status messages
    this->DeclareAbstractInputPort("panda_status", Value<lcmt_panda_status>());
    
    // Declare a periodic event that prints the message
    this->DeclarePeriodicEvent(
        drake::systems::Event<double>(
            drake::systems::Event<double>::TriggerType::kPeriodic),
        drake::systems::PublishEvent<double>(
            [this](const Context<double>& context,
                  const drake::systems::PublishEvent<double>&) {
              this->PrintMessage(context);
            }));
  }

 private:
  void PrintMessage(const Context<double>& context) const {
    // Get the input message
    const auto& input = this->EvalAbstractInput(context, 0);
    const auto& status = input->get_value<lcmt_panda_status>();

    // Print the message contents
    std::cout << "\n=== Received PANDA_STATUS message ===" << std::endl;
    std::cout << "Timestamp: " << status.utime << std::endl;
    std::cout << "Robot timestamp: " << status.robot_utime << std::endl;
    std::cout << "Number of joints: " << status.num_joints << std::endl;
    
    // Print joint positions
    std::cout << "Joint positions: [";
    for (size_t i = 0; i < status.joint_position.size(); ++i) {
      std::cout << status.joint_position[i];
      if (i < status.joint_position.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Print joint velocities
    std::cout << "Joint velocities: [";
    for (size_t i = 0; i < status.joint_velocity.size(); ++i) {
      std::cout << status.joint_velocity[i];
      if (i < status.joint_velocity.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // Print joint torques
    std::cout << "Joint torques: [";
    for (size_t i = 0; i < status.joint_torque.size(); ++i) {
      std::cout << status.joint_torque[i];
      if (i < status.joint_torque.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "Robot mode: " << static_cast<int>(status.robot_mode) << std::endl;
    std::cout << "Control success rate: " << status.control_command_success_rate << std::endl;
    std::cout << "=== End of message ===\n" << std::endl;
  }
};

}  // namespace systems
}  // namespace drake

int main() {
  // Create a diagram builder
  drake::systems::DiagramBuilder<double> builder;

  // Create and add the LCM interface system
  auto lcm = std::make_unique<drake::systems::lcm::LcmInterfaceSystem>();
  auto lcm_ptr = lcm.get();
  builder.AddSystem(std::move(lcm));

  // Create and add the LCM subscriber system
  auto subscriber = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_panda_status>(
          "PANDA_STATUS", lcm_ptr));

  // Create and add the printer system
  auto printer = builder.AddSystem<drake::systems::PandaStatusPrinter>();

  // Connect the subscriber output to the printer input
  builder.Connect(subscriber->get_output_port(), printer->get_input_port(0));

  // Build the diagram
  auto diagram = builder.Build();

  // Create a simulator
  drake::systems::Simulator<double> simulator(*diagram);

  // Set the simulator to run in real-time
  simulator.set_target_realtime_rate(1.0);

  std::cout << "Starting to monitor PANDA_STATUS messages..." << std::endl;
  std::cout << "Press Ctrl+C to exit" << std::endl;

  // Run the simulation
  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
} 