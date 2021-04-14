#ifndef RT_LOGGING_MACROS_HPP
#define RT_LOGGING_MACROS_HPP

#include <ros/console.h>

#define NAME_PREFIX "ros.rtcf"  // This is here since the compiling package of this header is usually not rtcf-package.

/**
 * @brief Macros for normal logging (NOT real-time safe).
 * These are forwarded directly to rosconsole. Since the logger name is used to identify the real-time component, no
 * NAMED-versions are available.
 *
 * NEVER EVER CALL THESE IN A REAL-TIME CRITICAL CONTEXT!
 */
// clang-format off
#define NON_RT_DEBUG(...)           ROS_LOG(ros::console::levels::Debug, std::string(NAME_PREFIX) + "." + this->getName(), __VA_ARGS__)
#define NON_RT_DEBUG_STREAM(args)   ROS_LOG_STREAM(ros::console::levels::Debug, std::string(NAME_PREFIX) + "." + this->getName(), args)

#define NON_RT_INFO(...)            ROS_LOG(ros::console::levels::Info, std::string(NAME_PREFIX) + "." + this->getName(), __VA_ARGS__)
#define NON_RT_INFO_STREAM(args)    ROS_LOG_STREAM(ros::console::levels::Info, std::string(NAME_PREFIX) + "." + this->getName(), args)

#define NON_RT_WARN(...)            ROS_LOG(ros::console::levels::Warn, std::string(NAME_PREFIX) + "." + this->getName(), __VA_ARGS__)
#define NON_RT_WARN_STREAM(args)    ROS_LOG_STREAM(ros::console::levels::Warn, std::string(NAME_PREFIX) + "." + this->getName(), args)

#define NON_RT_ERROR(...)           ROS_LOG(ros::console::levels::Error, std::string(NAME_PREFIX) + "." + this->getName(), __VA_ARGS__)
#define NON_RT_ERROR_STREAM(args)   ROS_LOG_STREAM(ros::console::levels::Error, std::string(NAME_PREFIX) + "." + this->getName(), args)

#define NON_RT_FATAL(...)           ROS_LOG(ros::console::levels::Fatal, std::string(NAME_PREFIX) + "." + this->getName(), __VA_ARGS__)
#define NON_RT_FATAL_STREAM(args)   ROS_LOG_STREAM(ros::console::levels::Fatal, std::string(NAME_PREFIX) + "." + this->getName(), args)
// clang-format on

/**
 * @brief Macros for real-time safe logging.
 * These macros allow real-time logging via rosconsole and friends. Real-time safe here means, that the code path
 * calling the log statement will not block due to ressource usage, allocation, etc.. The time consumed for processing
 * the log statement must still be taken into account in performance considerations. Internally, this uses a customized
 * version of OROCOS real-time logging. The actual output is done by a second thread which is low priority non-realtime.
 * Since the logger name is used to identify the real-time component, no NAMED-versions are available.
 */
// clang-format off
#define RT_DEBUG(str)           this->rtLogDebug(str)
#define RT_DEBUG_STREAM(args)   this->rtLogStream(ros::console::levels::Debug) << args

#define RT_INFO(str)           this->rtLogInfo(str)
#define RT_INFO_STREAM(args)   this->rtLogStream(ros::console::levels::Info) << args

#define RT_WARN(str)           this->rtLogWarn(str)
#define RT_WARN_STREAM(args)   this->rtLogStream(ros::console::levels::Warn) << args

#define RT_ERROR(str)           this->rtLogError(str)
#define RT_ERROR_STREAM(args)   this->rtLogStream(ros::console::levels::Error) << args

#define RT_FATAL(str)           this->rtLogFatal(str)
#define RT_FATAL_STREAM(args)   this->rtLogStream(ros::console::levels::Fatal) << args
// clang-format on

#endif  // RT_LOGGING_MACROS_HPP
