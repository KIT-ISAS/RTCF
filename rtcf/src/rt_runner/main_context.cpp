#include "main_context.hpp"

#include "rtcf/rtcf_extension.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rostopic.h>
#pragma GCC diagnostic pop

#include <iomanip>

MainContext::MainContext(std::string const& name)
    : TaskContext(name), port_iter_info_("iteration_info"), first_iteration_(true), count_delayed_iterations_(0) {}

bool MainContext::configureHook() {
    ROS_INFO("MainContext::configureHook() called");

    time_service_   = RTT::os::TimeService::Instance();
    period_desired_ = 1.0 / this->getPeriod();

    // create connection to ROS for iteration information
    this->ports()->addPort(port_iter_info_);
    // add a buffer to not miss something as this is important for debugging
    port_iter_info_.createStream(rtt_roscomm::topicBuffer("/rt_runner/" + port_iter_info_.getName(), 10));

    return true;
}

bool MainContext::startHook() {
    ROS_INFO("MainContext::startHook() called");
    clearStatistics();
    return true;
}

void MainContext::updateHook() {
    // save current time
    ros::Time callback_time = rtt_rosclock::host_now();  // this will resolve to ros::Time::now() or the sim time
    RTT::os::TimeService::nsecs start, end, delta;
    start                          = time_service_->getNSecs();
    RtcfExtension::last_timestamp_ = callback_time;

    // this does all the heavy lifting and calls all our components in order
    ROS_INFO("MainContext::updateHook() called");
    for (const auto& slave : slaves_) {
        slave->task_context->update();  // calls update on underlying activity
    }

    // calculate duration time and send it out
    end   = time_service_->getNSecs();
    delta = end - start;
    rtcf::IterationInformation info;
    info.stamp       = callback_time;
    info.duration_ns = (uint64_t)delta;
    port_iter_info_.write(info);

    // statistics for period duration
    if (first_iteration_) {
        first_iteration_ = false;
    } else {
        double period_duration = (callback_time - time_last_iteration_).toNSec();
        iter_period_acc_(period_duration);
        if (period_duration > 1.1 * period_desired_) {
            count_delayed_iterations_++;
        }
    }
    time_last_iteration_ = callback_time;
    // statistics for calculation duration
    iter_calculation_acc((double)delta);
}

void MainContext::stopHook() {
    ROS_INFO("MainContext::stopHook() called");
    // std::cout << "Main Context stopping !" << std::endl;
}

void MainContext::cleanupHook() {
    // std::cout << "MainContext cleaning up !" <<std::endl;
}

void MainContext::setSlaves(const RTOrder& slaves) {
    // std::cout << "Set slaves" << std::endl;
    slaves_ = slaves;
};

void MainContext::clearSlaves() {
    // std::cout << "Clear slaves" << std::endl;
    slaves_.clear();
};

std::string MainContext::generateStatisticsString() {
    double percentage_delayed = (double)count_delayed_iterations_ / acc::count(iter_period_acc_) * 100.0;

    std::stringstream ss;
    ss << std::setprecision(5);
    ss << "Performed " << acc::count(iter_calculation_acc) << " iterations." << std::endl;
    ss << "Calculation duration: "
       << "Mean " << acc::mean(iter_calculation_acc) << "us, Max " << acc::max(iter_calculation_acc) << " us, Min "
       << acc::min(iter_calculation_acc) << " us." << std::endl;
    ss << "Period duration: ";
    ss << "Mean " << acc::mean(iter_period_acc_) << "us, Max " << acc::max(iter_period_acc_) << " us, Min "
       << acc::min(iter_period_acc_) << " us, ";
    ss << percentage_delayed << " % delayed by more than 10 %.";

    return ss.str();
}

void MainContext::clearStatistics() {
    iter_period_acc_          = {};
    iter_calculation_acc      = {};
    first_iteration_          = true;
    count_delayed_iterations_ = 0;
}
