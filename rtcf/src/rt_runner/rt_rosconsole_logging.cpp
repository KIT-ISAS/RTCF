#include "rt_rosconsole_logging.hpp"

#include <ros/console.h>
#include <ros/service_manager.h>

#include <map>


RtRosconsoleLogging::RtRosconsoleLogging() : TaskContext("logger") {
    rt_memory_pool_.initialize(MEMORY_POOL_SIZE);

    this->addEventPort("log_port", log_port_);
}

bool RtRosconsoleLogging::configureHook() {
    // create default RT logger factory
    log4cpp::HierarchyMaintainer::set_category_factory(OCL::logging::Category::createOCLCategory);

    // make real-time loggers runtime-modifiable by intercepting rosconsole logger level callbacks
    // remove initial service handler
    ros::ServiceManager::instance()->unadvertiseService(nh_.resolveName("~set_logger_level"));
    // replace it with new service handler
    nh_.advertiseService("~set_logger_level", &RtRosconsoleLogging::setLoggerLevelCallback);

    // fetch all existing loggers from rosconsole and setup the corresponding real-time loggers
    std::map<std::string, ros::console::levels::Level> existing_loggers;
    ros::console::get_loggers(existing_loggers);
    for (const auto& pair : existing_loggers) {
        if (!setRTLoggerLevel(pair.first, pair.second)) {
            ROS_ERROR("RT logger could not be set up.");
        }
    }

    // connect the port of the ros-root logger (ROSCONSOLE_DEFAULT_NAME) to the this component
    OCL::logging::Category* category =
        dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance(ROSCONSOLE_DEFAULT_NAME));
    assert(category);
    RTT::ConnPolicy cp = RTT::ConnPolicy::buffer(500, RTT::ConnPolicy::LOCK_FREE, false, false);
    category->connectToLogPort(log_port_, cp);
    assert(log_port_.connected());

    return true;
}
bool RtRosconsoleLogging::startHook() { return true; }

void RtRosconsoleLogging::updateHook() { drainBuffer(); }

void RtRosconsoleLogging::stopHook() {
    log_port_.disconnect();
    assert(!log_port_.connected());
    drainBuffer();
}
void RtRosconsoleLogging::cleanupHook() { rt_memory_pool_.shutdown(); }

// This method is preferred for changing the log level programmatically as it changes rosconsole and the corresponding
// RT logger. Not real-time safe!
bool RtRosconsoleLogging::setLoggerLevel(const std::string& name, ros::console::levels::Level level) {
    if (ros::console::set_logger_level(name, level)) {
        ros::console::notifyLoggerLevelsChanged();
        if (RtRosconsoleLogging::setRTLoggerLevel(name, level)) {
            return true;
        }
        return false;
    }
    return false;
}

OCL::logging::Category* RtRosconsoleLogging::getLoggerInstance(const std::string& name){
    return dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance(name));
}

void RtRosconsoleLogging::drainBuffer() {
    OCL::logging::LoggingEvent event;
    // read all messages in buffer
    while (log_port_.read(event) == RTT::NewData) {
        // for each message, do the actual printing to console
        ros::console::levels::Level level = RtRosconsoleLogging::levelRTT2ROS(event.priority);
        std::string category              = RTT::makeString(event.categoryName);
        std::string message               = RTT::makeString(event.message);
        // timestamp is decided by ROS and therefore might be a bit delayed
        ROS_LOG(level, category, "%s", message.c_str());
    }
}

// This method is taken from ros_comm/clients/roscpp/src/libros/init.cpp,
// because it was not exported in the header.
bool RtRosconsoleLogging::setLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req,
                                                 roscpp::SetLoggerLevel::Response&) {
    // This is the original part
    std::transform(req.level.begin(), req.level.end(), req.level.begin(), (int (*)(int))std::toupper);

    ros::console::levels::Level level;
    if (req.level == "DEBUG") {
        level = ros::console::levels::Debug;
    } else if (req.level == "INFO") {
        level = ros::console::levels::Info;
    } else if (req.level == "WARN") {
        level = ros::console::levels::Warn;
    } else if (req.level == "ERROR") {
        level = ros::console::levels::Error;
    } else if (req.level == "FATAL") {
        level = ros::console::levels::Fatal;
    } else {
        return false;
    }

    bool success = ::ros::console::set_logger_level(req.logger, level);
    if (success) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // This is the extended part, where the real-time logger is changed
    success &= RtRosconsoleLogging::setRTLoggerLevel(req.logger, level);
    ROS_INFO("Logger changed via service call!");

    return success;
}

// This is not real-time safe
bool RtRosconsoleLogging::setRTLoggerLevel(const std::string& name, ros::console::Level level) {
    OCL::logging::Category* logger = dynamic_cast<OCL::logging::Category*>(&log4cpp::Category::getInstance(name));
    if (!logger) {
        return false;
    }

    log4cpp::Priority::Value prio = RtRosconsoleLogging::levelROS2RTT(level);
    assert(prio != log4cpp::Priority::NOTSET);
    logger->setPriority(prio);

    return true;
}

ros::console::Level RtRosconsoleLogging::levelRTT2ROS(const log4cpp::Priority::Value& prio) {
    switch (prio) {
        case log4cpp::Priority::DEBUG:
            return ros::console::levels::Debug;
        case log4cpp::Priority::INFO:
            return ros::console::levels::Info;
        case log4cpp::Priority::WARN:
            return ros::console::levels::Warn;
        case log4cpp::Priority::ERROR:
            return ros::console::levels::Error;
        case log4cpp::Priority::FATAL:
            return ros::console::levels::Fatal;
        default:
            return ros::console::levels::Count;
    }
}

log4cpp::Priority::Value RtRosconsoleLogging::levelROS2RTT(const ros::console::Level& prio) {
    switch (prio) {
        case ros::console::levels::Debug:
            return log4cpp::Priority::DEBUG;
        case ros::console::levels::Info:
            return log4cpp::Priority::INFO;
        case ros::console::levels::Warn:
            return log4cpp::Priority::WARN;
        case ros::console::levels::Error:
            return log4cpp::Priority::ERROR;
        case ros::console::levels::Fatal:
            return log4cpp::Priority::FATAL;
        default:
            return log4cpp::Priority::NOTSET;
    }
}
