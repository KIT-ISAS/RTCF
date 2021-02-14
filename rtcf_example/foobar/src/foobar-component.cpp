#include "foobar-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Foobar::Foobar(std::string const& name)
    : TaskContext(name),
      outPort("out_Port"),
      inPort_1("in_Port_1"),
      inPort_2("in_Port_2"),
      inPort_3("in_Port_3"),
      inPort_4("in_Port_4") {
    std::cout << "Foobar constructed !" << std::endl;
}

bool Foobar::configureHook(){
  std::cout << "Foobar configured !" <<std::endl;
    this->ports()->addPort(outPort);

    this->ports()->addPort(inPort_1);
    this->ports()->addPort(inPort_2);
    this->ports()->addPort(inPort_3);
    this->ports()->addPort(inPort_4);
    return true;
}

bool Foobar::startHook(){
  std::cout << "Foobar started !" <<std::endl;
  return true;
}

void Foobar::updateHook(){
}

void Foobar::stopHook() {
  std::cout << "Foobar executes stopping !" <<std::endl;
}

void Foobar::cleanupHook() {
  std::cout << "Foobar cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Foobar)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Foobar)
