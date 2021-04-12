#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

ParameterHandling::ParameterHandling(std::string const& name) : TaskContext(name) {
    std::cout << "ParameterHandling constructed !" << std::endl;
}

bool ParameterHandling::configureHook() {
    // set up (real-time) dynamic reconfigure
    dynamic_config.configure(getPrivateNodeHandle());

    // get static parameters from parameter server
    // this is NOT real-time safe
    int another_private_param;
    double public_param;
    bool success = true;
    success &= getPrivateNodeHandle().getParam("another_private_param", another_private_param);
    success &= getNodeHandle().getParam("public_param", public_param);

    std::cout << "ParameterHandling configured!" << std::endl;
    std::cout << "another_private_param=" << another_private_param << std::endl;
    std::cout << "public_param=" << public_param << std::endl;

    return true;
}

bool ParameterHandling::startHook() {
    std::cout << "ParameterHandling started !" << std::endl;
    return true;
}

void ParameterHandling::updateHook() {
    // get most recent dynamic parameters
    ExampleParameters config_data;
    dynamic_config.getValue(config_data);

    std::cout << "ParameterHandling executes updateHook with dynamic double parameter value "
              << config_data.double_param << "!" << std::endl;
}

void ParameterHandling::stopHook() { std::cout << "ParameterHandling executes stopping !" << std::endl; }

void ParameterHandling::cleanupHook() { std::cout << "ParameterHandling cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(ParameterHandling)
