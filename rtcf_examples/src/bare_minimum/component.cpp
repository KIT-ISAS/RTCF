#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

BareMinimum::BareMinimum(std::string const& name) : TaskContext(name) {
    std::cout << "BareMinimum constructed !" << std::endl;
}

bool BareMinimum::configureHook() {
    std::cout << "BareMinimum configured !" << std::endl;
    return true;
}

bool BareMinimum::startHook() {
    std::cout << "BareMinimum started !" << std::endl;
    return true;
}

void BareMinimum::updateHook() { std::cout << "BareMinimum executes updateHook !" << std::endl; }

void BareMinimum::stopHook() { std::cout << "BareMinimum executes stopping !" << std::endl; }

void BareMinimum::cleanupHook() { std::cout << "BareMinimum cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(BareMinimum)
