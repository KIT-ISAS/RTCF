#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtt/RTT.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>

#include "rt_runner_types.hpp"

class MainContext : public RTT::TaskContext {
  private:
    SlaveActivityVector slaves_;
    RTT::os::TimeService* time_service_ptr_;

  public:
    MainContext(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setSlaves(std::vector<RTT::extras::SlaveActivity*>);
    void clearSlaves();
};

#endif /* MAIN_CONTEXT_H */
