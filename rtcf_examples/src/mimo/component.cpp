#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

#include <rtcf/rt_logging_macros.hpp>

Mimo::Mimo(std::string const& name) :
    TaskContext(name),
    port_out1_("out1"),
    port_out2_("out2"),
    port_in1_("in1"),
    port_in2_("in2"),
    port_in3_("in3"),
    port_in4_("in4") {
    NON_RT_INFO("Mimo constructed !");
}

bool Mimo::configureHook() {
    NON_RT_INFO("Mimo configured !");

    this->ports()->addPort(port_in1_);
    this->ports()->addPort(port_in2_);
    this->ports()->addPort(port_in3_);
    this->ports()->addPort(port_in4_);

    this->ports()->addPort(port_out1_);
    this->ports()->addPort(port_out2_);

    return true;
}

bool Mimo::startHook() {
    NON_RT_INFO("Mimo started !");
    return true;
}

void Mimo::updateHook() {}

void Mimo::stopHook() { NON_RT_INFO("Mimo executes stopping !"); }

void Mimo::cleanupHook() { NON_RT_INFO("Mimo cleaning up !"); }

ORO_CREATE_COMPONENT(Mimo)
