#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

Identity::Identity(std::string const& name) : TaskContext(name) { std::cout << "Identity constructed !" << std::endl; }

bool Identity::configureHook() {
    std::cout << "Identity configured !" << std::endl;
    return true;
}

bool Identity::startHook() {
    std::cout << "Identity started !" << std::endl;
    return true;
}

void Identity::updateHook() {
    std::cout << "Identity executes updateHook !" << std::endl;

    // just forward the message
    if (port_in_.read(msg) == RTT::NewData) {
        port_out_.write(msg);
        port_in_.clear();
    }
}

void Identity::stopHook() { std::cout << "Identity executes stopping !" << std::endl; }

void Identity::cleanupHook() { std::cout << "Identity cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(Identity)
