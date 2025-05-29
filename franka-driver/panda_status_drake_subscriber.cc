#include "panda_status_drake_subscriber.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace manipulation {
namespace panda {

using systems::BasicVector;
using systems::Context;

PandaStatusReceiver::PandaStatusReceiver(int num_joints)
    : num_joints_(num_joints) {
  this->DeclareAbstractInputPort("lcmt_panda_status", Value<lcmt_panda_status>{});

  this->DeclareVectorOutputPort("position", num_joints_,
      &PandaStatusReceiver::CalcLcmOutput<&lcmt_panda_status::joint_position>);

  this->DeclareVectorOutputPort("velocity", num_joints_,
      &PandaStatusReceiver::CalcLcmOutput<&lcmt_panda_status::joint_velocity>);

  this->DeclareVectorOutputPort("torque", num_joints_,
      &PandaStatusReceiver::CalcLcmOutput<&lcmt_panda_status::joint_torque>);
}

PandaStatusReceiver::~PandaStatusReceiver() = default;

const systems::OutputPort<double>&
PandaStatusReceiver::get_position_output_port() const {
  return LeafSystem<double>::get_output_port(0);
}

const systems::OutputPort<double>&
PandaStatusReceiver::get_velocity_output_port() const {
  return LeafSystem<double>::get_output_port(1);
}

const systems::OutputPort<double>&
PandaStatusReceiver::get_torque_output_port() const {
  return LeafSystem<double>::get_output_port(2);
}

void PandaStatusReceiver::PrintStatus(const Context<double>& context) const {
  const auto& status = get_input_port().Eval<lcmt_panda_status>(context);
  
  std::cout << "\n=== Panda Status Message ===" << std::endl;
  std::cout << "Number of joints: " << status.num_joints << std::endl;
  
  if (status.num_joints > 0) {
    std::cout << "\nJoint Positions:" << std::endl;
    for (int i = 0; i < status.num_joints; ++i) {
      std::cout << "  Joint " << i << ": " << status.joint_position[i] << " rad" << std::endl;
    }
    
    std::cout << "\nJoint Velocities:" << std::endl;
    for (int i = 0; i < status.num_joints; ++i) {
      std::cout << "  Joint " << i << ": " << status.joint_velocity[i] << " rad/s" << std::endl;
    }
    
    std::cout << "\nJoint Torques:" << std::endl;
    for (int i = 0; i < status.num_joints; ++i) {
      std::cout << "  Joint " << i << ": " << status.joint_torque[i] << " N⋅m" << std::endl;
    }
  }
  std::cout << "==========================\n" << std::endl;
}

template <std::vector<double> lcmt_panda_status::*field_ptr>
void PandaStatusReceiver::CalcLcmOutput(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& status = get_input_port().Eval<lcmt_panda_status>(context);

  if (status.num_joints == 0) {
    output->get_mutable_value().setZero();
    return;
  }

  const auto& field = status.*field_ptr;
  DRAKE_THROW_UNLESS(status.num_joints == num_joints_);
  DRAKE_THROW_UNLESS(static_cast<int>(field.size()) == num_joints_);

  // Print status message whenever this method is called
  std::cout << "\n=== Panda Status Message ===" << std::endl;
  std::cout << "Context time: " << context.get_time() << " seconds" << std::endl;
  std::cout << "Number of joints: " << status.num_joints << std::endl;
  
  if (status.num_joints > 0) {
    std::cout << "\nJoint Positions:" << std::endl;
    for (int i = 0; i < status.num_joints; ++i) {
      std::cout << "  Joint " << i << ": " << status.joint_position[i] << " rad" << std::endl;
    }
    
    std::cout << "\nJoint Velocities:" << std::endl;
    for (int i = 0; i < status.num_joints; ++i) {
      std::cout << "  Joint " << i << ": " << status.joint_velocity[i] << " rad/s" << std::endl;
    }
    
    std::cout << "\nJoint Torques:" << std::endl;
    for (int i = 0; i < status.num_joints; ++i) {
      std::cout << "  Joint " << i << ": " << status.joint_torque[i] << " N⋅m" << std::endl;
    }
  }
  std::cout << "==========================\n" << std::endl;

  output->get_mutable_value() = Eigen::Map<const Eigen::VectorXd>(
      field.data(), num_joints_);
}

}  // namespace panda
}  // namespace manipulation
}  // namespace drake
