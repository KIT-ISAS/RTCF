#include "main_context.hpp"

#include "rtcf/rtcf_extension.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rostopic.h>
#pragma GCC diagnostic pop

MainContext::MainContext(std::string const& name) : TaskContext(name), port_iter_info_("iteration_info") {}

bool MainContext::configureHook() {
    ROS_INFO("MainContext::configureHook() called");

    time_service_ = RTT::os::TimeService::Instance();

    // create connection to ROS for iteration information
    this->ports()->addPort(port_iter_info_);
    // add a buffer to not miss something as this is important for debugging
    port_iter_info_.createStream(rtt_roscomm::topicBuffer("/rt_runner/" + port_iter_info_.getName(), 10));

    return true;
}

bool MainContext::startHook() {
    ROS_INFO("MainContext::startHook() called");
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

    // provide it to the system in some way (with timestamp, duration, optionally min, optionally max)
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
