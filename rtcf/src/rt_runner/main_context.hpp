#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtcf_msgs/IterationInfo.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#include <rtt/extras/SlaveActivity.hpp>
OROCOS_HEADERS_END

#include "rt_runner_types.hpp"
#include "timing_analysis.hpp"
class MainContext : public RTT::TaskContext {
  private:
    RTOrder slaves_;

    TimingAnalysis timing_analysis_;
    RTT::OutputPort<rtcf_msgs::IterationInfo> port_iter_info_;

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
