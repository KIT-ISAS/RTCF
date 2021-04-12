#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

ParameterHandling::ParameterHandling(std::string const& name) : TaskContext(name) {
    std::cout << "ParameterHandling constructed !" << std::endl;
}

bool ParameterHandling::configureHook() {
    std::cout << "ParameterHandling configured !" << std::endl;
    dynamic_config.configure(getPrivateNodeHandle());
    return true;
}

bool ParameterHandling::startHook() {
    std::cout << "ParameterHandling started !" << std::endl;
    return true;
}

void ParameterHandling::updateHook() {
    ExampleParameters config_data;
    dynamic_config.getValue(config_data);
    std::cout << "ParameterHandling executes updateHook with dynamic double parameter value "
              << config_data.double_param << "!" << std::endl;
}

void ParameterHandling::stopHook() { std::cout << "ParameterHandling executes stopping !" << std::endl; }

void ParameterHandling::cleanupHook() { std::cout << "ParameterHandling cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(ParameterHandling)
