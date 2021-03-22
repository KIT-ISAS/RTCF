#include "timing_analysis.hpp"

#include <iomanip>
#include <sstream>

TimingAnalysis::TimingAnalysis() {
    time_service_ = RTT::os::TimeService::Instance();
    reset();
}

void TimingAnalysis::configure(double period_desired_s, double period_margin) {
    period_threshold_ = (1.0 + period_margin) * period_desired_s * 1'000'000'000;  // to ns
    period_margin_    = period_margin;
}

void TimingAnalysis::reset() {
    acc_iter_period_          = {};
    acc_iter_calculation_     = {};
    first_iteration_          = true;
    count_delayed_iterations_ = 0;
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
    acc_iter_calculation_(delta_calculation_);

    // determince period duration
    if (first_iteration_) {
        first_iteration_ = false;
    } else {
        RTT::os::TimeService::nsecs period_duration = (time_iteration_ - time_last_iteration_).toNSec();
        acc_iter_period_(period_duration);
        if (period_duration > period_threshold_) {
            count_delayed_iterations_++;
        }
    }
    time_last_iteration_ = time_iteration_;
}

std::string TimingAnalysis::generateStatisticsString() const {
    std::stringstream ss;

    // number of iterations
    ss << "Performed " << acc::count(acc_iter_calculation_) / 1000 << "k iterations." << std::endl;

    // calculation duration
    ss << "Calculation duration: ";
    ss << "Mean " << acc::mean(acc_iter_calculation_) / 1000 << "us, "
       << "Max " << acc::max(acc_iter_calculation_) / 1000 << " us, "
       << "Min " << acc::min(acc_iter_calculation_) / 1000 << " us." << std::endl;

    // period duration
    ss << "Period duration: ";
    ss << "Mean " << acc::mean(acc_iter_period_) / 1000 << "us, "
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
