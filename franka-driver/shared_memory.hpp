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
      : data(alloc), ee_wrench(alloc), data_ready(false) {}

  boost::interprocess::interprocess_mutex mutex;
  boost::interprocess::interprocess_condition cond_var;

  ShmemVector data;       // 14-element [pos + vel]
  ShmemVector ee_wrench;  // 6-element [Fx, Fy, Fz, Tx, Ty, Tz]
  bool data_ready;
};

#endif // SHARED_MEMORY_HPP
