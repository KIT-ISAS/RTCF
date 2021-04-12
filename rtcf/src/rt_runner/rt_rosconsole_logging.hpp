#ifndef RT_ROSONSOLE_LOGGING_HPP
#define RT_ROSONSOLE_LOGGING_HPP

#include <rtt/InputPort.hpp>
#include <rtt/TaskContext.hpp>

class RtRosconsoleLogging : public RTT::TaskContext {
  public:
    RtRosconsoleLogging();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    RTT::InputPort<OCL::logging::LoggingEvent> log_port;

    // TODO:
    // - setup log4cpp
    // - create ros.rtcf logger with properties active in ROS
    // - intercept rosconsole-loggerlevel callbacks
    // - create event port for ros.rtcf
    // - push everything that arrives to rosconsole
};

#endif  // RT_ROSONSOLE_LOGGING_HPP
