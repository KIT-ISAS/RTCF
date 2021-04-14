#include "component.hpp"

#include <iostream>
#include <rtcf/rt_logging_macros.hpp>
#include <rtt/Component.hpp>

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
