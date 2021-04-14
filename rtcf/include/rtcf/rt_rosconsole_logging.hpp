#ifndef RT_ROSONSOLE_LOGGING_HPP
#define RT_ROSONSOLE_LOGGING_HPP

#include <ros/ros.h>
#include <roscpp/SetLoggerLevel.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
// this is a workaround for old log4cpp version that do not compile in C++17 due to usage of deprecated features
#define throw(...)
#include <log4cpp/HierarchyMaintainer.hh>
#undef throw

#include <ocl/Category.hpp>
#include <ocl/LoggingEvent.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>
OROCOS_HEADERS_END

#include "tlsf_memory_pool.hpp"

class RtRosconsoleLogging : public RTT::TaskContext {
  public:
    static RtRosconsoleLogging& getInstance();

    static constexpr size_t MEMORY_POOL_SIZE = 1024 * 1024;  // 1 MByte

    static bool setLoggerLevel(const std::string& name, ros::console::levels::Level level);

    static OCL::logging::Category* getLoggerInstance(const std::string& name);

    static ros::console::Level levelRTT2ROS(const log4cpp::Priority::Value& prio);
    static log4cpp::Priority::Value levelROS2RTT(const ros::console::Level& prio);

  protected:
    RtRosconsoleLogging();

    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    void drainBuffer();

    RTT::InputPort<OCL::logging::LoggingEvent> log_port_;
    TlsfMemoryPool rt_memory_pool_;
    ros::ServiceServer logger_level_service_;

    static bool setLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response&);
    static bool setRTLoggerLevel(const std::string& name, ros::console::levels::Level);

  private:
    static std::shared_ptr<RtRosconsoleLogging> instance;

    // TODO:
    // - create macros in RTCF extension
    // DONE:
    // - unload logger gracefully
    // - test
    // - load logger at the right places
    // - remove code from RTCF extension
    // - push everything that arrives to rosconsole
    // - create event port for ros.rtcf
    // - setup log4cpp
    // - intercept rosconsole-loggerlevel callbacks
    // - create ros.rtcf logger with properties active in ROS
};

#endif  // RT_ROSONSOLE_LOGGING_HPP
