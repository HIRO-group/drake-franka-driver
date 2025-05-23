#include <memory>
#include <iostream>

#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcmt_panda_status.hpp"

#include "panda_status_drake_subscriber.h"

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

  // Create and add the status receiver system
  auto status_receiver = builder.AddSystem<drake::manipulation::panda::PandaStatusReceiver>(7);  // 7 joints for Panda

  // Connect the subscriber output to the status receiver input
  builder.Connect(subscriber->get_output_port(), status_receiver->get_input_port(0));

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
  
  // Create a periodic event to print status
  simulator.Initialize();

  while (true) {
    simulator.AdvanceTo(simulator.get_context().get_time() + 0.1);
    const auto& context = simulator.get_context();
    const auto& receiver_context = status_receiver->GetMyContextFromRoot(context);
    status_receiver->PrintStatus(receiver_context);

  }


  return 0;
} 