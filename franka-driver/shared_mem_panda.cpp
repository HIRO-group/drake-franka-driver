#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/lcmt_panda_status.hpp"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/primitives/vector_log_sink.h"

#include "panda_status_drake_subscriber.h"
#include "shared_memory.hpp"  // your Boost.Interprocess shared memory header

namespace bip = boost::interprocess;

int main() {
  // Initialize shared memory
  std::cout<<"if ever run say yellow"<<std::endl;
  bip::shared_memory_object::remove("MySharedMemory");
  bip::managed_shared_memory segment(bip::create_only, "MySharedMemory", 65536);
  const auto alloc = SharedMemoryData::ShmemAllocator(segment.get_segment_manager());
  SharedMemoryData* shm = segment.construct<SharedMemoryData>("SharedData")(alloc);

  bip::shared_memory_object::remove("torque_shared_data");
  bip::managed_shared_memory segment_torque(bip::create_only, "torque_shared_data", 65536);
  const auto alloc_torque = SharedMemoryData::ShmemAllocator(segment_torque.get_segment_manager());
  SharedMemoryData* shm_torque = segment_torque.construct<SharedMemoryData>("SharedData")(alloc_torque);

  // Build Drake diagram
  drake::systems::DiagramBuilder<double> builder;
  auto lcm = drake::lcm::DrakeLcm();
  auto subscriber = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_panda_status>(
          "PANDA_STATUS", &lcm));

  auto status_receiver = builder.AddSystem<drake::manipulation::panda::PandaStatusReceiver>(7);
  builder.Connect(subscriber->get_output_port(), status_receiver->get_input_port(0));

  auto position_logger = builder.AddSystem<drake::systems::VectorLogSink<double>>(7);  // 7 joints
  position_logger->set_name("position_logger");

  // Connect the status receiver's position output to the logger
  builder.Connect(status_receiver->get_position_output_port(), position_logger->get_input_port());

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  simulator.Initialize();
  simulator.get_mutable_context().SetTime(0.0);

  std::cout << "Monitoring PANDA_STATUS and writing to shared memory...\n";

  const double end_time = 10.0;
  const double step = 0.01;  // 100 Hz
  const auto& status_context = diagram->GetSubsystemContext(*status_receiver, simulator.get_context());


  while (simulator.get_context().get_time()<end_time) {
    simulator.AdvanceTo(simulator.get_context().get_time() + step);

    const auto& status_context = diagram->GetSubsystemContext(*status_receiver, simulator.get_context());
    std::vector<double> wrench;
    {
      bip::scoped_lock<bip::interprocess_mutex> lock(shm_torque->mutex);
      wrench.assign(shm_torque->ee_wrench.begin(), shm_torque->ee_wrench.end());
    }

    std::cout << "End-effector wrench [Fx, Fy, Fz, Tx, Ty, Tz]: ";
    for (size_t i = 0; i < wrench.size(); ++i) {
      std::cout << wrench[i];
      if (i < wrench.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    if (status_receiver->get_input_port(0).HasValue(status_context)) {
        std::cout << "Input port has value!" << std::endl;
    } else {
        std::cout << "Input port has NO VALUE!" << std::endl;
    }

    // Get subsystem context for receiver

    const auto& pos_vec = status_receiver->get_position_output_port().Eval(status_context);
    const auto& vel_vec = status_receiver->get_velocity_output_port().Eval(status_context);

    // Concatenate pos, vel, and wrench into a single vector (size 20)
    std::vector<double> combined(20);
    for (int i = 0; i < 7; ++i) {
      combined[i] = pos_vec[i];
      combined[i + 7] = vel_vec[i];
    }
    for (int i = 0; i < 6; ++i) {
      combined[14 + i] = wrench[i];
    }


    // Write to shared memory
    std::cout << "Writing to shared memory [pos+vel]: ";
    for (size_t i = 0; i < combined.size(); ++i) {
    std::cout << combined[i];
    if (i < combined.size() - 1) std::cout << ", ";
    }
    std::cout<< "Death" << std::endl;
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
  }

  // Get the correct subsystem context for the loggers
  auto& logger_context = diagram->GetMutableSubsystemContext(*position_logger, &simulator.get_mutable_context());

  // Get all the logged data using the logger's context
  const auto& log = position_logger->GetLog(logger_context);
  const auto& times = log.sample_times();
  const auto& data = log.data();

  // Write all logged data points to the logger files
  std::ofstream logger_file("position_logger_data.txt");
  logger_file << "Time (s),Joint1,Joint2,Joint3,Joint4,Joint5,Joint6,Joint7" << std::endl;

  for (int i = 0; i < times.size(); ++i) {
    // Write position logger data
    logger_file << std::fixed << std::setprecision(6) << times[i] << ",";
    for (int j = 0; j < 7; ++j) {
      logger_file << std::fixed << std::setprecision(6) << data(j, i);
      if (j < 6) logger_file << ",";
    }
    logger_file << std::endl;


  }

  bip::shared_memory_object::remove("MySharedMemory");
  return 0;
}
