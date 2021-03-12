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

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(BareMinimum)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(BareMinimum)
