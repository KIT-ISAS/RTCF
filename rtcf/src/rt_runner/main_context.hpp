#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtcf/IterationInformation.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#pragma GCC diagnostic pop
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>

#include "rt_runner_types.hpp"

class MainContext : public RTT::TaskContext {
  private:
    RTOrder slaves_;

    RTT::os::TimeService* time_service_;
    RTT::OutputPort<rtcf::IterationInformation> port_iter_info_;

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
