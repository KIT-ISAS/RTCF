#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

#include <rtcf/rt_logging_macros.hpp>

BareMinimum::BareMinimum(std::string const& name) : TaskContext(name) { NON_RT_INFO("BareMinimum constructed !"); }

bool BareMinimum::configureHook() {
    NON_RT_INFO("BareMinimum configured !");
    return true;
}

bool BareMinimum::startHook() {
    NON_RT_INFO("BareMinimum started !");
    return true;
}

void BareMinimum::updateHook() {
    // put your periodic payload here
}

void BareMinimum::stopHook() { NON_RT_INFO("BareMinimum executes stopping !"); }

void BareMinimum::cleanupHook() { NON_RT_INFO("BareMinimum cleaning up !"); }

ORO_CREATE_COMPONENT(BareMinimum)
