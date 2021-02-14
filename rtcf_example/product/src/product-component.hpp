#ifndef OROCOS_PRODUCT_COMPONENT_HPP
#define OROCOS_PRODUCT_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Product : public RTT::TaskContext{
  public:
    Product(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::OutputPort<double> outPort;

    RTT::InputPort<double> inPort_1;
    RTT::InputPort<double> inPort_2;
    RTT::InputPort<double> inPort_3;
    RTT::InputPort<double> inPort_4;

    double in_msg_1;
    double in_msg_2;
    double in_msg_3;
    double in_msg_4;

    double out_msg;
};
#endif
