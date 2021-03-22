#ifndef TIMING_ANALYSIS_HPP
#define TIMING_ANALYSIS_HPP

// #include <boost/accumulators/accumulators.hpp>
// #include <boost/accumulators/statistics/max.hpp>
// #include <boost/accumulators/statistics/mean.hpp>
// #include <boost/accumulators/statistics/min.hpp>
// #include <boost/accumulators/statistics/variance.hpp>
#include <iostream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <rtt_rosclock/rtt_rosclock.h>
#pragma GCC diagnostic pop

// namespace acc = boost::accumulators;

template <class T>
class Accumulator {
  public:
    Accumulator() { reset(); }

    void operator()(const T& val) {
        count++;
        sum += val;
        min = std::min(min, val);
        max = std::max(max, val);
    }

    void reset() {
        this->count = 0;
        this->sum   = 0;
        this->min   = +std::numeric_limits<T>::max();
        this->max   = +std::numeric_limits<T>::min();
    }

    size_t getCount() const { return count; }
    T getSum() const { return sum; }
    double getMean() const { return (double)sum / (double)count; };
    T getMin() const { return min; }
    T getMax() const { return max; }

  private:
    size_t count;
    T sum;
    T min;
    T max;
};

class TimingAnalysis {
  public:
    using time_t = uint64_t;
    // using DurationAccumulator =
    //    acc::accumulator_set<time_t, acc::features<acc::tag::count, acc::tag::min, acc::tag::max, acc::tag::mean>>;
    using DurationAccumulator = Accumulator<time_t>;

    TimingAnalysis();
    void configure(double period, double outlier_threshold);
    void reset();
    const ros::Time& start();
    void stop();

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
    RTT::os::TimeService::nsecs delta_calculation_;
    ros::Time time_iteration_;
    ros::Time time_last_iteration_;
    bool first_iteration_;

    // overly delayed count
    size_t count_delayed_iterations_;
    RTT::os::TimeService::nsecs period_threshold_;
    double period_margin_;
};

std::ostream& operator<<(std::ostream& os, const TimingAnalysis& t);

#endif  // TIMING_ANALYSIS_HPP
