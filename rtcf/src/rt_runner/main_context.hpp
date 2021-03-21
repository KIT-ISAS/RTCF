#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtcf/IterationInformation.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#pragma GCC diagnostic pop
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>

#include "rt_runner_types.hpp"

namespace acc = boost::accumulators;

class MainContext : public RTT::TaskContext {
  public:
    using DurationAccumulator = acc::accumulator_set<
        double, acc::features<acc::tag::count, acc::tag::min, acc::tag::max, acc::tag::mean(acc::immediate)>>;

    MainContext(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setSlaves(const RTOrder& order);
    void clearSlaves();

    std::string generateStatisticsString();
    void clearStatistics();

  private:
    RTOrder slaves_;

    RTT::os::TimeService* time_service_;
    RTT::OutputPort<rtcf::IterationInformation> port_iter_info_;

    // statistics
    DurationAccumulator iter_period_acc_;
    DurationAccumulator iter_calculation_acc;
    ros::Time time_last_iteration_;
    bool first_iteration_;
    RTT::os::TimeService::nsecs period_desired_;
    size_t count_delayed_iterations_;
};

#endif /* MAIN_CONTEXT_H */
