#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include <ros/ros.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/Port.hpp>
OROCOS_HEADERS_END

#include "rtcf/rt_logging_macros.hpp"
#include "rtcf/rt_rosconsole_logging.hpp"

class RtcfExtension {
    // allow some classes access to private parameters
    // reason: public setter methods are tempting for the component-developers
    friend class ComponentContainer;
    friend class MainContext;
    friend class RTRunner;

  private:
    // by using pointers the user will not accidentally use the node handle in the component constructor
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr nh_private_;

    static double frequency_;
    // static double period_;
    static ros::Time last_timestamp_;
    static size_t iteration_;

    OCL::logging::Category* logger_;

  public:
    RtcfExtension() {
        last_timestamp_ = ros::Time(0);
        iteration_      = 0;
    };
    virtual ~RtcfExtension(){};

    ros::NodeHandle& getNodeHandle() const { return *nh_; }
    ros::NodeHandle& getPrivateNodeHandle() const { return *nh_private_; }

    const ros::Time& getTime() const { return last_timestamp_; }
    const size_t& getIteration() const { return iteration_; }
    const double& getFrequency() const { return frequency_; }
    // const double& getPeriod() const { return period_; }

    void rtLogDebug(const RTT::rt_string& message) { logger_->debug(message); }
    void rtLogInfo(const RTT::rt_string& message) { logger_->info(message); }
    void rtLogWarn(const RTT::rt_string& message) { logger_->warn(message); }
    void rtLogError(const RTT::rt_string& message) { logger_->error(message); }
    void rtLogFatal(const RTT::rt_string& message) { logger_->fatal(message); }

    OCL::logging::CategoryStream rtLogStream(ros::console::Level level) {
        return logger_->getRTStream(RtRosconsoleLogging::levelROS2RTT(level));
    }

    void configureTestLogger() {
        // this logger is not real-time safe and only meant for testing purposes
        ROS_WARN("Component logger was set to non real-time safe test mode.");
        log4cpp::HierarchyMaintainer::set_category_factory(OCL::logging::Category::createOCLCategory);
        logger_ = dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance("test"));
    }
};

#endif /* RTCF_EXTENSION_H */
