#include "main_context.hpp"

#include "rtcf/rtcf_extension.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include <rtt_ros/rtt_ros.h>
#include <rtt_roscomm/rostopic.h>
#pragma GCC diagnostic pop

MainContext::MainContext(std::string const& name) : TaskContext(name), port_iter_info_("iteration_info") {}

bool MainContext::configureHook() {
    ROS_DEBUG("MainContext::configureHook() called");

    timing_analysis_.configure(this->getPeriod());

    // create connection to ROS for iteration information
    this->ports()->addPort(port_iter_info_);
    rtt_ros::import("rtt_rtcf_msgs");
    // add a buffer to not miss something as this is important for debugging
    port_iter_info_.createStream(rtt_roscomm::topicBuffer("/rt_runner/" + port_iter_info_.getName(), 10));

    return true;
}

bool MainContext::startHook() {
    ROS_DEBUG("MainContext::startHook() called");
    timing_analysis_.reset();
    return true;
}

void MainContext::updateHook() {
    // save current time
    RtcfExtension::last_timestamp_ = timing_analysis_.start();

    // this does all the heavy lifting and calls all our components in order
    for (const auto& slave : slaves_) {
        slave->task_context->update();  // calls update on underlying activity
    }

    // calculate duration time and send it out
    timing_analysis_.stop();

    // publish debugging information
    rtcf_msgs::IterationInformation info;
    info.stamp       = timing_analysis_.getIterationStamp();
    info.duration_ns = (uint64_t)timing_analysis_.getCalculationDuration();
    port_iter_info_.write(info);
}

void MainContext::stopHook() { ROS_DEBUG("MainContext::stopHook() called"); }

void MainContext::cleanupHook() {
    ROS_DEBUG("MainContext::cleanUp() called");
    ROS_INFO_STREAM(timing_analysis_);
}

void MainContext::setSlaves(const RTOrder& slaves) { slaves_ = slaves; }

void MainContext::clearSlaves() { slaves_.clear(); }
