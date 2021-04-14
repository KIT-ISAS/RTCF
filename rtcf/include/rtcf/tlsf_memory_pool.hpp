#ifndef TLSF_MEMORY_POOL
#define TLSF_MEMORY_POOL

#include <cstddef>

/**
 * @brief This class manages an TLSF (two-level segregated fit) memory pool.
 * The TLSF implementation is provided by OROCOS and this class is taken from
 * orocos_toolchain/ocl/bin/deployer-funcs.cpp/.hpp
 * Unfortunately, it was not compiled into a library there and hence had to be copied by us.
 *
 * The first instance of this object will be used by oro_rt_malloc and friends.
 */
class TlsfMemoryPool {
  public:
    /// Create default object (no allocation to pool)
    TlsfMemoryPool();
    /// Shutdown and deallocate memory pool (if necessary)
    ~TlsfMemoryPool();

    /** Initialize the default TLSF memory pool
     **
     ** @param memSize Size of memory pool in bytes
     ** @return true if successful
     ** @post if succesful then rtMem!=0 otherwise and rtMem=0
     */
    bool initialize(const size_t memSize);

    /** Shutdown the default TLSF memory pool
     ** @post rtMem=0 and the default TLSF memory pool is no longer available
     */
    void shutdown();

  protected:
    /// Memory allocated for the pool
    void* rtMem;
};

#endif  // TLSF_MEMORY_POOL