#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

Identity::Identity(std::string const& name) : TaskContext(name), port_out_("out_port"), port_in_("in_port") {
    std::cout << "Identity constructed !" << std::endl;
}

bool Identity::configureHook() {
    std::cout << "Identity configured !" << std::endl;

    this->ports()->addPort(port_in_);
    this->ports()->addPort(port_out_);

    return true;
}

bool Identity::startHook() {
    std::cout << "Identity started !" << std::endl;
    return true;
}

void Identity::updateHook() {
    std::cout << "Identity executes updateHook !" << std::endl;

    // just forward the message
    if (port_in_.read(msg_) == RTT::NewData) {
        port_out_.write(msg_);
    }
}

void Identity::stopHook() { std::cout << "Identity executes stopping !" << std::endl; }

void Identity::cleanupHook() { std::cout << "Identity cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(Identity)
