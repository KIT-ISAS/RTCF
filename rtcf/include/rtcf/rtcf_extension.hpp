#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include <ros/ros.h>

#include <rtt/Port.hpp>

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

    OCL::logging::Category* logger_;

  public:
    RtcfExtension() { last_timestamp_ = ros::Time(0); };
    virtual ~RtcfExtension(){};

    const ros::NodeHandle& getNodeHandle() const { return *nh_; }
    const ros::NodeHandle& getPrivateNodeHandle() const { return *nh_private_; }

    const ros::Time& getTime() const { return last_timestamp_; }
    const double& getFrequency() const { return frequency_; }
    // const double& getPeriod() const { return period_; }

    void RtLogDebug(const RTT::rt_string& message) { logger_->debug(message); }
    void RtLogInfo(const RTT::rt_string& message) { logger_->info(message); }
    void RtLogWarn(const RTT::rt_string& message) { logger_->warn(message); }
    void RtLogError(const RTT::rt_string& message) { logger_->error(message); }
    void RtLogFatal(const RTT::rt_string& message) { logger_->fatal(message); }
    
    OCL::logging::CategoryStream RtLogStream(ros::console::Level level){
          return logger_->getRTStream(RtRosconsoleLogging::levelROS2RTT(level));
    }
};

#endif /* RTCF_EXTENSION_H */
