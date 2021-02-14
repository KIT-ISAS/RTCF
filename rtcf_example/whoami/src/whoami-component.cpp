#include "whoami-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Whoami::Whoami(std::string const& name) : TaskContext(name), outPort("out_Port"), inPort_1("in_Port_1"), inPort_2("in_Port_2"), inPort_3("in_Port_3"), inPort_4("in_Port_4") {
  std::cout << "Whoami constructed !" <<std::endl;
}

bool Whoami::configureHook(){
    std::cout << "Whoami configured !" << std::endl;

    this->ports()->addPort(outPort);

    this->ports()->addPort(inPort_1);
    this->ports()->addPort(inPort_2);
    this->ports()->addPort(inPort_3);
    this->ports()->addPort(inPort_4);
    return true;
}

bool Whoami::startHook(){
  std::cout << "Whoami started !" <<std::endl;
  return true;
}

void Whoami::updateHook(){
    std::cout << node_handle_ptr_->getNamespace() << std::endl;
}

void Whoami::stopHook() {
  std::cout << "Whoami executes stopping !" <<std::endl;
}

void Whoami::cleanupHook() {
  std::cout << "Whoami cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Whoami)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Whoami)
