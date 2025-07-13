#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#endif //SHARED_DATA_H
// shared_data.hpp
#pragma once

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

namespace bip = boost::interprocess;

struct SharedMemoryData {
    typedef bip::allocator<double, bip::managed_shared_memory::segment_manager> ShmemAllocator;
    typedef bip::vector<double, ShmemAllocator> SharedVector;

    SharedMemoryData(const ShmemAllocator& alloc)
        : data(alloc), ee_wrench(alloc),data_ready(false) {}

    bip::interprocess_mutex mutex;
    bip::interprocess_condition cond_var;

    bool data_ready;
    SharedVector data;
    ShmemVector ee_wrench;  // new: end-effector wrench (size 6)

};