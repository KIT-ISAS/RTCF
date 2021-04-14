#include "component.hpp"

#include <iostream>
#include <rtcf/rt_logging_macros.hpp>
#include <rtt/Component.hpp>

Identity::Identity(std::string const& name) : TaskContext(name), port_out_("out_port"), port_in_("in_port") {
    NON_RT_INFO("Identity constructed !");
}

bool Identity::configureHook() {
    NON_RT_INFO("Identity configured !");

    this->ports()->addPort(port_in_);
    this->ports()->addPort(port_out_);

    return true;
}

bool Identity::startHook() {
    NON_RT_INFO("Identity started !");
    return true;
}

void Identity::updateHook() {
    // just forward the message
    if (port_in_.read(msg_) == RTT::NewData) {
        port_out_.write(msg_);
    }
}

void Identity::stopHook() { NON_RT_INFO("Identity executes stopping !"); }

void Identity::cleanupHook() { NON_RT_INFO("Identity cleaning up !"); }

ORO_CREATE_COMPONENT(Identity)
