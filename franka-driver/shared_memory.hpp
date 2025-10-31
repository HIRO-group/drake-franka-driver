#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <vector>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

struct SharedMemoryData {
  typedef double ValueType;
  typedef boost::interprocess::allocator<ValueType, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
  typedef boost::interprocess::vector<ValueType, ShmemAllocator> ShmemVector;

  SharedMemoryData(const ShmemAllocator& alloc)
      : data(alloc), ee_wrench(alloc), gripper_cmd(alloc), data_ready(false), gripper_cmd_ready(false) {}

  boost::interprocess::interprocess_mutex mutex;
  boost::interprocess::interprocess_condition cond_var;

  ShmemVector data;       // 14-element [pos + vel]
  ShmemVector ee_wrench;  // 6-element [Fx, Fy, Fz, Tx, Ty, Tz]
  // Gripper command vector layout (fixed positions):
  // [0]=command_code (1=open, 2=close, 3=vac_on, 4=vac_off)
  // [1]=width (m), [2]=speed (m/s), [3]=force (N),
  // [4]=eps_inner (m), [5]=eps_outer (m),
  // [6]=strength (0..1), [7]=timeout_ms
  ShmemVector gripper_cmd;
  bool data_ready;
  bool gripper_cmd_ready;
};

#endif // SHARED_MEMORY_HPP
