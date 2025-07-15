#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <limits>

#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcmt_panda_status.hpp"
#include "drake/systems/primitives/vector_log_sink.h"

#include "panda_status_drake_subscriber.h"
#include "shared_memory.hpp"  // your Boost.Interprocess shared memory header

namespace bip = boost::interprocess;

int main() {
  
  // Create a diagram builder
  drake::systems::DiagramBuilder<double> builder;

  // Initialize shared memory for writing combined data
  bip::shared_memory_object::remove("MySharedMemory");
  bip::managed_shared_memory segment(bip::create_only, "MySharedMemory", 65536);
  const auto alloc = SharedMemoryData::ShmemAllocator(segment.get_segment_manager());
  SharedMemoryData* shm = segment.construct<SharedMemoryData>("SharedData")(alloc);

  // Initialize shared memory for reading wrench data
  bip::managed_shared_memory segment_torque(bip::open_or_create, "torque_shared_data", 65536);
  const auto alloc_torque = SharedMemoryData::ShmemAllocator(segment_torque.get_segment_manager());
  SharedMemoryData* shm_torque = nullptr;
  
  // Try to find existing torque shared memory data
  try {
    shm_torque = segment_torque.find<SharedMemoryData>("SharedData").first;
    if (shm_torque == nullptr) {
      // Create the shared memory data if it doesn't exist
      shm_torque = segment_torque.construct<SharedMemoryData>("SharedData")(alloc_torque);
      std::cout << "Created new torque shared memory data" << std::endl;
    } else {
      std::cout << "Found existing torque shared memory data" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Torque shared memory initialization failed: " << e.what() << std::endl;
    shm_torque = nullptr;
  }


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

  // Create and add a logger for the position output
  auto position_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(7);  // 7 joints
  position_logger->set_name("position_logger");

  // Connect the status receiver's position output to the logger
  builder.Connect(status_receiver->get_position_output_port(), position_logger->get_input_port());

  // Build the diagram
  auto diagram = builder.Build();

  // Create a simulator
  drake::systems::Simulator<double> simulator(*diagram);

  // Set the simulator to run in real-time
  simulator.set_target_realtime_rate(1.0);

  // Create and open CSV file
  std::ofstream csv_file("joint_angles.csv");
  if (!csv_file.is_open()) {
    std::cerr << "Failed to open joint_angles.csv" << std::endl;
    return 1;
  }

  // Write header to CSV
  csv_file << "timestamp,joint1,joint2,joint3,joint4,joint5,joint6,joint7" << std::endl;

  // Create and open text file for vector logger data
  std::ofstream logger_file("vector_logger_data.txt");
  if (!logger_file.is_open()) {
    std::cerr << "Failed to open vector_logger_data.txt" << std::endl;
    return 1;
  }

  // Write header to logger file
  logger_file << "Time (s),Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7" << std::endl;

  std::cout << "Starting to monitor PANDA_STATUS messages..." << std::endl;
  std::cout << "Writing joint angles to joint_angles.csv" << std::endl;
  std::cout << "Writing vector logger data to vector_logger_data.txt" << std::endl;
  std::cout << "Press Ctrl+C to exit" << std::endl;

  // Run the simulation
  simulator.Initialize();

  // Get the context for the status receiver
  auto& receiver_context = diagram->GetMutableSubsystemContext(*status_receiver, &simulator.get_mutable_context());

  // Set up a periodic event to write to CSV
  simulator.get_mutable_context().SetTime(0.0);
  const double dt = 0.01;  // 10ms sampling rate
  const double end_time = 10.0;  // 10 seconds total

  while (simulator.get_context().get_time() < end_time) {
    // Get current joint positions
    const auto& positions = status_receiver->get_position_output_port().Eval(receiver_context);
    const auto& velocities = status_receiver->get_velocity_output_port().Eval(receiver_context);

    // Read wrench data from torque shared memory
    std::vector<double> wrench(6, 0.0);  // Default to zeros if no wrench data
    if (shm_torque != nullptr) {
      try {
        bip::scoped_lock<bip::interprocess_mutex> lock(shm_torque->mutex);
        if (!shm_torque->ee_wrench.empty()) {
          wrench.assign(shm_torque->ee_wrench.begin(), shm_torque->ee_wrench.end());
        }
      } catch (const std::exception& e) {
        std::cerr << "Failed to read wrench from shared memory: " << e.what() << std::endl;
      }
    }

    // Concatenate pos, vel, and wrench into a single vector (size 20)
    std::vector<double> combined(20);
    for (int i = 0; i < 7; ++i) {
      combined[i] = positions[i];        // Indices 0-6: joint positions
      combined[i + 7] = velocities[i];   // Indices 7-13: joint velocities
    }
    for (int i = 0; i < 6; ++i) {
      combined[14 + i] = wrench[i];      // Indices 14-19: end-effector wrench
    }

    // Write to shared memory
    std::cout << "Writing to shared memory [pos+vel+wrench]: ";
    for (size_t i = 0; i < combined.size(); ++i) {
      std::cout << combined[i];
      if (i < combined.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    
    {
      bip::scoped_lock<bip::interprocess_mutex> lock(shm->mutex);
      shm->data.assign(combined.begin(), combined.begin() + 14);        // pos + vel
      shm->ee_wrench.assign(combined.begin() + 14, combined.end());     // wrench
      shm->data_ready = true;
      shm->cond_var.notify_one();
    }

    // Optional delay for lower CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // Write to CSV
    csv_file << std::fixed << std::setprecision(6) << simulator.get_context().get_time() << ",";
    for (int i = 0; i < 7; ++i) {
      csv_file << std::fixed << std::setprecision(6) << positions[i];
      if (i < 6) csv_file << ",";
    }
    csv_file << std::endl;

    // Advance simulation
    simulator.AdvanceTo(simulator.get_context().get_time() + dt);
  }

  // Get the correct subsystem context for the logger
  auto& logger_context = diagram->GetMutableSubsystemContext(*position_logger, &simulator.get_mutable_context());

  // Get all the logged data using the logger's context
  const auto& log = position_logger->GetLog(logger_context);
  const auto& times = log.sample_times();
  const auto& data = log.data();

  // Write all logged data points to the logger file
  for (size_t i = 0; i < times.size(); ++i) {
    logger_file << std::fixed << std::setprecision(6) << times[i] << ",";
    for (int j = 0; j < 7; ++j) {
      logger_file << std::fixed << std::setprecision(6) << data(j, i);
      if (j < 6) logger_file << ",";
    }
    logger_file << std::endl;
  }

  csv_file.close();
  logger_file.close();
  return 0;
}