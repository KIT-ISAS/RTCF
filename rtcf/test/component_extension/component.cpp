#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

ExtensionTest::ExtensionTest(std::string const& name) : TaskContext(name) {
    std::cout << "ExtensionTest constructed !" << std::endl;
}

bool ExtensionTest::configureHook() {
    std::cout << "ExtensionTest configured !" << std::endl;

    // read a parameter and write it to two different locations
    std::string value = "";
    this->getPrivateNodeHandle().getParam("parameter_in", value);
    this->getPrivateNodeHandle().setParam("parameter_out", value);
    this->getNodeHandle().setParam("parameter_out", value);
    this->getNodeHandle().setParam("/absolute/parameter_out", value);

    return true;
}

bool ExtensionTest::startHook() {
    std::cout << "ExtensionTest started !" << std::endl;
    return true;
}

void ExtensionTest::updateHook() { std::cout << "ExtensionTest executes updateHook !" << std::endl; }

void ExtensionTest::stopHook() { std::cout << "ExtensionTest executes stopping !" << std::endl; }

void ExtensionTest::cleanupHook() { std::cout << "ExtensionTest cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(ExtensionTest)
