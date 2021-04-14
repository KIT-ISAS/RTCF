#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

#include <rtcf/rt_logging_macros.hpp>

ExtensionTest::ExtensionTest(std::string const& name) : TaskContext(name) { NON_RT_INFO("ExtensionTest constructed "); }

bool ExtensionTest::configureHook() {
    NON_RT_INFO("ExtensionTest configured !");

    // read a parameter and write it to two different locations
    std::string value = "";
    this->getPrivateNodeHandle().getParam("parameter_in", value);
    this->getPrivateNodeHandle().setParam("parameter_out", value);
    this->getNodeHandle().setParam("parameter_out", value);
    this->getNodeHandle().setParam("/absolute/parameter_out", value);

    return true;
}

bool ExtensionTest::startHook() {
    NON_RT_INFO("ExtensionTest started !");
    return true;
}

void ExtensionTest::updateHook() {}

void ExtensionTest::stopHook() { NON_RT_INFO("ExtensionTest executes stopping !"); }

void ExtensionTest::cleanupHook() { NON_RT_INFO("ExtensionTest cleaning up !"); }

ORO_CREATE_COMPONENT(ExtensionTest)
