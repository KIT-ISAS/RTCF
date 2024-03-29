#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

Logging::Logging(std::string const& name) : TaskContext(name) {
    NON_RT_INFO("Non-RT logging works in constructor.");
    // RT_INFO("RT logging does NOT work in constructor.");
}

bool Logging::configureHook() {
    // non real-time logging
    NON_RT_DEBUG("Test non-RT debug");
    NON_RT_DEBUG_STREAM("Test non-RT debug stream with number " << 12345);
    NON_RT_INFO("Test non-RT info");
    NON_RT_INFO_STREAM("Test non-RT info stream with number " << 12345);
    NON_RT_WARN("Test non-RT warn");
    NON_RT_WARN_STREAM("Test non-RT warn stream with number " << 12345);
    NON_RT_ERROR("Test non-RT error");
    NON_RT_ERROR_STREAM("Test non-RT error stream with number " << 12345);
    NON_RT_FATAL("Test non-RT fatal");
    NON_RT_FATAL_STREAM("Test non-RT fatal stream with number " << 12345);

    return true;
}

bool Logging::startHook() { return true; }

void Logging::updateHook() {
    // real-time logging
    RT_DEBUG("Test RT debug");
    RT_DEBUG_STREAM("Test RT debug stream with number " << 12345);
    RT_INFO("Test RT info");
    RT_INFO_STREAM("Test RT info stream with number " << 12345);
    RT_WARN("Test RT warn");
    RT_WARN_STREAM("Test RT warn stream with number " << 12345);
    RT_ERROR("Test RT error");
    RT_ERROR_STREAM("Test RT error stream with number " << 12345);
    RT_FATAL("Test RT fatal");
    RT_FATAL_STREAM("Test RT fatal stream with number " << 12345);

    // illegal things (will crash the program on purpose)
    // NON_RT_INFO("Boom!");
    // NON_RT_INFO_STREAM("Boom with stream!");
}

void Logging::stopHook() {}

void Logging::cleanupHook() {}

ORO_CREATE_COMPONENT(Logging)
