#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

ParameterHandling::ParameterHandling(std::string const& name) : TaskContext(name) {
    NON_RT_INFO("ParameterHandling constructed !");
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

    NON_RT_INFO("ParameterHandling configured!");
    NON_RT_INFO_STREAM("another_private_param=" << another_private_param);
    NON_RT_INFO_STREAM("public_param=" << public_param);

    return true;
}

bool ParameterHandling::startHook() {
    NON_RT_INFO("ParameterHandling started !");
    return true;
}

void ParameterHandling::updateHook() {
    // get most recent dynamic parameters
    ExampleParameters config_data;
    dynamic_config.getValue(config_data);

    RT_INFO_STREAM("ParameterHandling executes updateHook with dynamic double parameter value "
                   << config_data.double_param << "!");
}

void ParameterHandling::stopHook() { NON_RT_INFO("ParameterHandling executes stopping !"); }

void ParameterHandling::cleanupHook() { NON_RT_INFO("ParameterHandling cleaning up !"); }

ORO_CREATE_COMPONENT(ParameterHandling)
