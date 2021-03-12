#include "product-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>

Product::Product(std::string const& name) : TaskContext(name){
  std::cout << "Product constructed !" <<std::endl;
}

bool Product::configureHook() {
    std::cout << "Product configured !" << std::endl;

    this->ports()
        ->addPort("out_Port", outPort)
        .doc("Output Port, here write our data to.");

    this->ports()->addPort("in_Port_1", inPort_1).doc("Input Port 1 for product.");
    this->ports()->addPort("in_Port_2", inPort_2).doc("Input Port 2 for product.");
    this->ports()->addPort("in_Port_3", inPort_3).doc("Input Port 3 for product.");
    this->ports()->addPort("in_Port_4", inPort_4).doc("Input Port 4 for product.");
    return true;
}

bool Product::startHook(){
  std::cout << "Product started !" <<std::endl;
  return true;
}

void Product::updateHook(){
  std::cout << "Product executes updateHook !" <<std::endl;

    out_msg = 1.0;

    if (inPort_1.read(in_msg_1)==RTT::NewData) {
        out_msg *= in_msg_1;
    }

    if (inPort_2.read(in_msg_2)==RTT::NewData) {
        out_msg *= in_msg_2;
    }

    if (inPort_3.read(in_msg_3)==RTT::NewData) {
        out_msg *= in_msg_3;
    }

    if (inPort_4.read(in_msg_4)==RTT::NewData) {
        out_msg *= in_msg_4;
    }

    outPort.write(out_msg);
}

void Product::stopHook() {
  std::cout << "Product executes stopping !" <<std::endl;
}

void Product::cleanupHook() {
  std::cout << "Product cleaning up !" <<std::endl;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Product)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Product)
