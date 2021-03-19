#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtt/RTT.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>

#include "rt_runner_types.hpp"

class MainContext : public RTT::TaskContext {
  private:
    RTOrder slaves_;
    RTT::os::TimeService* time_service_ptr_;

  public:
    MainContext(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setSlaves(const RTOrder& order);
    void clearSlaves();
};

#endif /* MAIN_CONTEXT_H */
