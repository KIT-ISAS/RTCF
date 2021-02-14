#include "sum-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Sum::Sum(std::string const& name) : TaskContext(name), outPort("out_Port"), inPort_1("in_Port_1"), inPort_2("in_Port_2"), inPort_3("in_Port_3"), inPort_4("in_Port_4") {
  std::cout << "Sum constructed !" <<std::endl;
}

bool Sum::configureHook(){
    std::cout << "Sum configured !" << std::endl;

    this->ports()->addPort(outPort);

    this->ports()->addPort(inPort_1);
    this->ports()->addPort(inPort_2);
    this->ports()->addPort(inPort_3);
    this->ports()->addPort(inPort_4);
    return true;
}

bool Sum::startHook(){
  std::cout << "Sum started !" <<std::endl;
  return true;
}

void Sum::updateHook(){
  std::cout << "Sum executes updateHook !" <<std::endl;
    std::cout << node_handle_ptr_->getNamespace() << std::endl;

    out_msg.data = 0.0;

    if (inPort_1.read(in_msg_1)==RTT::NewData) {
        out_msg.data += in_msg_1.data;
    }

    if (inPort_2.read(in_msg_2)==RTT::NewData) {
        out_msg.data += in_msg_2.data;
    }

    if (inPort_3.read(in_msg_3)==RTT::NewData) {
        out_msg.data += in_msg_3.data;
    }

    if (inPort_4.read(in_msg_4)==RTT::NewData) {
        out_msg.data += in_msg_4.data;
    }

    outPort.write(out_msg);
}

void Sum::stopHook() {
  std::cout << "Sum executes stopping !" <<std::endl;
}

void Sum::cleanupHook() {
  std::cout << "Sum cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Sum)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Sum)
