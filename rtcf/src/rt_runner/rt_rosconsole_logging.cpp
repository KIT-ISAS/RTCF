#include "rt_rosconsole_logging.hpp"

#include <ros/console.h>

// this is a workaround for old log4cpp version that do not compile in C++17 due to usage of deprecated features
#define throw(...)
#undef throw

#include <log4cpp/HierarchyMaintainer.hh>
#include <ocl/Category.hpp>

// check if real-time allocation is available in this build
#ifndef OS_RT_MALLOC
#error "Real-time logging needs real-time safe memory allocation!"
#endif

RtRosconsoleLogging::RtRosconsoleLogging() {
    log4cpp::HierarchyMaintainer::set_category_factory(OCL::logging::Category::createOCLCategory);
}

bool RtRosconsoleLogging::configureHook() { ROS_INFO("Logging configureHook()"); }
bool RtRosconsoleLogging::startHook() {}
void RtRosconsoleLogging::updateHook() {}
void RtRosconsoleLogging::stopHook() {}
void RtRosconsoleLogging::cleanupHook() {}