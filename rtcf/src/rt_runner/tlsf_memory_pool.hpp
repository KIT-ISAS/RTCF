#ifndef TLSF_MEMORY_POOL
#define TLSF_MEMORY_POOL

#include <cstdint>

/**
 * @brief This class manages an TLSF (two-level segregated fit) memory pool.
 * The TLSF implementation is provided by OROCOS and this class is taken from
 * orocos_toolchain/ocl/bin/deployer-funcs.cpp/.hpp
 * Unfortunately, it was not compiled into a library there and hence had to be copied by us.
 */
class TLSFMemoryPool {
  public:
    /// Create default object (no allocation to pool)
    TLSFMemoryPool();
    /// Shutdown and deallocate memory pool (if necessary)
    ~TLSFMemoryPool();

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