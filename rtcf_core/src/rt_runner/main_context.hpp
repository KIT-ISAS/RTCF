#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtt/RTT.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>

#include "ros/ros.h"

class MainContext : public RTT::TaskContext {
  private:
  public:
    MainContext(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setSlaves(std::vector<RTT::extras::SlaveActivity*>);
    void clearSlaves();

    std::vector<RTT::extras::SlaveActivity*> slaves_;

    RTT::os::TimeService* time_service_ptr;
};

#endif /* MAIN_CONTEXT_H */
