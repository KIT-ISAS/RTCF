#ifndef PREFAULTING_HPP
#define PREFAULTING_HPP

#include <alloca.h>
#include <malloc.h>
#include <ros/console.h>
#include <sys/mman.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/base/ExecutableInterface.hpp>
OROCOS_HEADERS_END

class PrefaultingExecutable : public RTT::base::ExecutableInterface {
  public:
    PrefaultingExecutable() : heap_size_(0), stack_size_(0) {}
    PrefaultingExecutable(size_t heap_size, size_t stack_size) : heap_size_(heap_size), stack_size_(stack_size) {}

    // this is called from main thread context
    bool execute() override {
        // see
        // https://rt.wiki.kernel.org/index.php/Threaded_RT-application_with_memory_locking_and_stack_handling_example
        // for some details

        // memory locking
        {
            if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
                ROS_ERROR_STREAM("Call to mlockall() failed with error '" << strerror(errno) << "'.");
            }
            mallopt(M_TRIM_THRESHOLD, -1);  // Turn off malloc trimming.
            mallopt(M_MMAP_MAX, 0);         // Turn off mmap usage.
        }

        // pre-fault heap
        if (heap_size_ > 0) {
            uint8_t* heap_buffer = (uint8_t*)malloc(heap_size_);
            if (!heap_buffer) {
                ROS_ERROR("Could not pre-fault heap.");
            } else {
                memset(heap_buffer, 0xbb, heap_size_);
                free(heap_buffer);
            }
        }

        // pre-fault stack (Yes, stack memory can be allocated dynamically!)
        if (stack_size_ > 0) {
            uint8_t* stack_buffer = (uint8_t*)alloca(stack_size_);
            if (!stack_buffer) {
                ROS_ERROR("Could not pre-fault stack.");
            } else {
                memset(stack_buffer, 0xcc, stack_size_);
                // stack memory is freed automatically
            }
        }

        ROS_WARN_STREAM("Pre-faulting was performed as configured.");

        return false;  // remove this from the queue
    }

  private:
    size_t heap_size_;
    size_t stack_size_;
};

#endif  // PREFAULTING_HPP
