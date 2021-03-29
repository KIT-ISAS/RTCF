#ifndef TIMING_ANALYSIS_HPP
#define TIMING_ANALYSIS_HPP

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <rtt_rosclock/rtt_rosclock.h>
#pragma GCC diagnostic pop

namespace acc = boost::accumulators;

class TimingAnalysis {
  public:
    using time_t = uint64_t;
    using DurationAccumulator =
        acc::accumulator_set<time_t, acc::features<acc::tag::count, acc::tag::min, acc::tag::max, acc::tag::mean>>;

    TimingAnalysis();
    void configure(double period, double outlier_threshold = 0.1, size_t iterations_to_ignore = 1000);
    void reset();
    const ros::Time& start();
    void stop();

    const DurationAccumulator& getAccuPeriodDuration() const { return acc_iter_period_; };
    const DurationAccumulator& getAccuCalculationDuration() const { return acc_iter_calculation_; };

    const ros::Time& getIterationStamp() const { return time_iteration_; }
    const RTT::os::TimeService::nsecs& getCalculationDuration() const { return delta_calculation_; }

    std::string generateStatisticsString() const;

  private:
    // statistics
    DurationAccumulator acc_iter_period_;
    DurationAccumulator acc_iter_calculation_;

    // time source
    RTT::os::TimeService* time_service_;

    // period duration calculation
    RTT::os::TimeService::nsecs start_calculation_;
    RTT::os::TimeService::nsecs start_calculation_last_;
    RTT::os::TimeService::nsecs delta_calculation_;
    ros::Time time_iteration_;
    size_t remaining_iterations_to_ignore_;

    // overly delayed count
    size_t count_delayed_iterations_;
    RTT::os::TimeService::nsecs period_threshold_;
    double period_margin_;

    // number of ignored iterations at start
    size_t iterations_to_ignore_;
};

std::ostream& operator<<(std::ostream& os, const TimingAnalysis& t);

#endif  // TIMING_ANALYSIS_HPP
