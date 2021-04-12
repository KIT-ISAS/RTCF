#include "tlsf_memory_pool.hpp"

#include <rtt/os/tlsf/tlsf.h>

TLSFMemoryPool::TLSFMemoryPool() : rtMem(0) {}

TLSFMemoryPool::~TLSFMemoryPool() { shutdown(); }

bool TLSFMemoryPool::initialize(const size_t memSize) {
    if (0 != rtMem) return false;    // avoid double init
    if (0 >= memSize) return false;  // invalid size

    // don't calloc() as is first thing TLSF does.
    rtMem = malloc(memSize);
    assert(0 != rtMem);
    const size_t freeMem = init_memory_pool(memSize, rtMem);
    if ((size_t)-1 == freeMem) {
        ROS_ERROR_SREAM(std::dec << "Invalid memory pool size of " << memSize
                                 << " bytes (TLSF has a several kilobyte overhead).");
        free(rtMem);
        rtMem = 0;
        return false;
    }
    ROS_INFO_STREAM("Real-time memory: " << freeMem << " bytes free of " << memSize << " allocated.");
    return true;
}

void TLSFMemoryPool::shutdown() {
    if (0 != rtMem) {
        const size_t overhead = get_overhead_size(rtMem);
        ROS_INFO_STREAM(std::dec << "TLSF bytes allocated=" << get_pool_size(rtMem) << " overhead=" << overhead
                                 << " max-used=" << (get_max_size(rtMem) - overhead)
                                 << " still-allocated=" << (get_used_size(rtMem) - overhead));
        destroy_memory_pool(rtMem);
        free(rtMem);
        rtMem = 0;
    }
}

#endif  // TLSF_MEMORY_POOL