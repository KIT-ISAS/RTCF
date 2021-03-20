#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

Mimo::Mimo(std::string const& name)
    : TaskContext(name),
      port_out1_("out1"),
      port_out2_("out2"),
      port_in1_("in1"),
      port_in2_("in2"),
      port_in3_("in3"),
      port_in4_("in4") {
    std::cout << "Mimo constructed !" << std::endl;
}

bool Mimo::configureHook() {
    std::cout << "Mimo configured !" << std::endl;

    this->ports()->addPort(port_in1_);
    this->ports()->addPort(port_in2_);
    this->ports()->addPort(port_in3_);
    this->ports()->addPort(port_in4_);

    this->ports()->addPort(port_out1_);
    this->ports()->addPort(port_out2_);

    return true;
}

bool Mimo::startHook() {
    std::cout << "Mimo started !" << std::endl;
    return true;
}

void Mimo::updateHook() { std::cout << "Mimo executes updateHook !" << std::endl; }

void Mimo::stopHook() { std::cout << "Mimo executes stopping !" << std::endl; }

void Mimo::cleanupHook() { std::cout << "Mimo cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(Mimo)
