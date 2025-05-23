#pragma once

#include <vector>
#include <iostream>

#include "drake/systems/framework/leaf_system.h"
#include "drake/lcmt_panda_status.hpp"

namespace drake {
namespace manipulation {
namespace panda {

class PandaStatusReceiver : public systems::LeafSystem<double> {
 public:
  explicit PandaStatusReceiver(int num_joints);
  ~PandaStatusReceiver() override;

  const systems::OutputPort<double>& get_position_output_port() const;
  const systems::OutputPort<double>& get_velocity_output_port() const;
  const systems::OutputPort<double>& get_torque_output_port() const;

  // Prints the current status message to stdout
  void PrintStatus(const systems::Context<double>& context) const;

 private:
  template <std::vector<double> lcmt_panda_status::*field_ptr>
  void CalcLcmOutput(const systems::Context<double>& context,
                     systems::BasicVector<double>* output) const;

  int num_joints_;
};

}  // namespace panda
}  // namespace manipulation
}  // namespace drake
