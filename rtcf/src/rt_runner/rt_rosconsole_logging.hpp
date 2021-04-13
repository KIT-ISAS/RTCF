#ifndef RT_ROSONSOLE_LOGGING_HPP
#define RT_ROSONSOLE_LOGGING_HPP

#include <ros/ros.h>
#include <roscpp/SetLoggerLevel.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#define throw(...)
#include <ocl/LoggingEvent.hpp>
#undef throw
#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
#pragma GCC diagnostic pop

#include "tlsf_memory_pool.hpp"

class RtRosconsoleLogging : public RTT::TaskContext {
  public:
    RtRosconsoleLogging();
    static constexpr size_t MEMORY_POOL_SIZE = 1024 * 512;  // 512 kByte

    static bool setLoggerLevel(const std::string& name, ros::console::levels::Level level);

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    void drainBuffer();

    RTT::InputPort<OCL::logging::LoggingEvent> log_port_;
    TlsfMemoryPool rt_memory_pool_;
    ros::NodeHandle nh_;

    static bool setLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response&);
    static bool setRTLoggerLevel(const std::string& name, ros::console::levels::Level);

    static ros::console::Level levelRTT2ROS(const log4cpp::Priority::Value& prio);
    static log4cpp::Priority::Value levelROS2RTT(const ros::console::Level& prio);

    // TODO:
    // - remove code from RTCF extension
    // - load logger at the right places
    // - test
    // - create macros in RTCF extension
    // DONE:
    // - push everything that arrives to rosconsole
    // - create event port for ros.rtcf
    // - setup log4cpp
    // - intercept rosconsole-loggerlevel callbacks
    // - create ros.rtcf logger with properties active in ROS
};

#endif  // RT_ROSONSOLE_LOGGING_HPP
