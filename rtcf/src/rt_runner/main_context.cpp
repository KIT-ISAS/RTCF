#include "main_context.hpp"

#include <rtt_rosclock/rtt_rosclock.h>

MainContext::MainContext(std::string const& name) : TaskContext(name) {}

bool MainContext::configureHook() {
    ROS_INFO("MainContext::configureHook() called");

    time_service_ptr_ = RTT::os::TimeService::Instance();
    time_service_ptr_->enableSystemClock(true);
    return true;
}

bool MainContext::startHook() {
    ROS_INFO("MainContext::startHook() called");
    return true;
}

void MainContext::updateHook() {
    ros::Time callback_time = rtt_rosclock::host_now();
    // get start time
    // provide the current time to all components
    // - options: iterate over all slaves and enter information in RTCF extensions
    // - add static variables in RTCF extension

    // this does all the heavy lifting and calls all our components in order
    ROS_INFO("MainContext::updateHook() called");
    for (const auto& slave : slaves_) {
        slave->task_context->update();  // calls update on underlying activity
    }

    // get end time
    // calculate difference
    // provide it to the system in some way (with timestamp, duration, optionally min, optionally max)
    // idea: use an orocos port that is connected via a stream to ROS
    // add some statistics for print during shutdown? / issue a warning (RT-safe)
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
