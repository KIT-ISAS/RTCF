#include "rtcf/tlsf_memory_pool.hpp"

#include <ros/ros.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/os/tlsf/tlsf.h>
OROCOS_HEADERS_END

#include <cstdlib>

// check if real-time allocation is available in this build
#ifndef OS_RT_MALLOC
#error "Logging needs real-time allocation. Check build options of OROCOS!"
#endif
#ifndef ORO_MEMORY_POOL
#error "Logging needs real-time memory pool. Check build options of OROCOS!"
#endif

TlsfMemoryPool::TlsfMemoryPool() : rtMem(0) {}

TlsfMemoryPool::~TlsfMemoryPool() { shutdown(); }

bool TlsfMemoryPool::initialize(const size_t memSize) {
    if (0 != rtMem) return false;    // avoid double init
    if (0 >= memSize) return false;  // invalid size

    // don't calloc() as is first thing TLSF does.
    rtMem = malloc(memSize);
    assert(0 != rtMem);
    memset(rtMem, 0xaa, memSize);  // pre-fault
    const size_t freeMem = init_memory_pool(memSize, rtMem);
    if ((size_t)-1 == freeMem) {
        ROS_ERROR_STREAM("Invalid memory pool size of " << memSize << " bytes (TLSF has a several kilobyte overhead).");
        free(rtMem);
        rtMem = 0;
        return false;
    }
    ROS_DEBUG_STREAM("Real-time memory: " << freeMem << " bytes free of " << memSize << " allocated.");
    return true;
}

void TlsfMemoryPool::shutdown() {
    if (0 != rtMem) {
        const size_t overhead = get_overhead_size(rtMem);
        ROS_DEBUG_STREAM("TLSF bytes allocated=" << get_pool_size(rtMem) << " overhead=" << overhead
                                                 << " max-used=" << (get_max_size(rtMem) - overhead)
                                                 << " still-allocated=" << (get_used_size(rtMem) - overhead));
        if (get_used_size(rtMem) - overhead != 0) {
            ROS_ERROR("Not all memory TLSF memory pool has been freed.");
        }
        destroy_memory_pool(rtMem);
        free(rtMem);
        rtMem = 0;
    }
}
