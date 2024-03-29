#include "timing_analysis.hpp"

#include <iomanip>
#include <sstream>

TimingAnalysis::TimingAnalysis() {
    time_service_ = RTT::os::TimeService::Instance();
    reset();
}

void TimingAnalysis::configure(double period_desired_s, double period_margin, size_t iterations_to_ignore) {
    period_threshold_     = (1.0 + period_margin) * period_desired_s * 1'000'000'000;  // to ns
    period_margin_        = period_margin;
    iterations_to_ignore_ = iterations_to_ignore;
    reset();
}

void TimingAnalysis::reset() {
    acc_iter_period_                = {};
    acc_iter_calculation_           = {};
    count_delayed_iterations_       = 0;
    remaining_iterations_to_ignore_ = iterations_to_ignore_ + 1;  // one more to get a valid timestamp of last cycle
}

const ros::Time& TimingAnalysis::start() {
    // We use two different clocks:
    // 1. For the duration we use RTT' built-in clock as this is a monotonic clock.
    // 2. For the time instant we use rtt_rosclock which either maps to sim time or the non-monotonic real-time clock.
    time_iteration_    = rtt_rosclock::host_now();
    start_calculation_ = time_service_->getNSecs();
    return time_iteration_;
}

void TimingAnalysis::stop() {
    // determine calculation duration
    delta_calculation_ = time_service_->getNSecs() - start_calculation_;

    if (remaining_iterations_to_ignore_ > 0) {
        remaining_iterations_to_ignore_--;
    } else {
        // store calculation duration
        acc_iter_calculation_(delta_calculation_);

        // determine period duration
        RTT::os::TimeService::nsecs period_duration =
            std::max((RTT::os::TimeService::nsecs)0, start_calculation_ - start_calculation_last_);
        acc_iter_period_(period_duration);
        if (period_duration > period_threshold_) {
            count_delayed_iterations_++;
        }
    }
    start_calculation_last_ = start_calculation_;
}

std::string TimingAnalysis::generateStatisticsString() const {
    std::stringstream ss;

    // number of iterations
    ss << "Performed " << acc::count(acc_iter_calculation_) << " iterations "
       << "(first " << iterations_to_ignore_ << " ignored) since last start." << std::endl;

    // calculation duration
    ss << "Calculation duration: ";
    ss << "Mean " << (time_t)(acc::mean(acc_iter_calculation_) / 1000) << " us, "
       << "Max " << acc::max(acc_iter_calculation_) / 1000 << " us, "
       << "Min " << acc::min(acc_iter_calculation_) / 1000 << " us." << std::endl;

    // period duration
    ss << "Period duration: ";
    ss << "Mean " << (time_t)(acc::mean(acc_iter_period_) / 1000) << " us, "
       << "Max " << acc::max(acc_iter_period_) / 1000 << " us, "
       << "Min " << acc::min(acc_iter_period_) / 1000 << " us, ";
    double percentage_delayed = (double)count_delayed_iterations_ / acc::count(acc_iter_period_) * 100.0;
    ss << std::setprecision(3) << percentage_delayed << " % delayed "
       << "by more than " << (int)(period_margin_ * 100) << " %.";

    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const TimingAnalysis& t) {
    os << t.generateStatisticsString();
    return os;
}
