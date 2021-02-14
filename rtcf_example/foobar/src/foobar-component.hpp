#ifndef OROCOS_FOOBAR_COMPONENT_HPP
#define OROCOS_FOOBAR_COMPONENT_HPP

#include <rtt/RTT.hpp>

class Foobar : public RTT::TaskContext{
  public:
    Foobar(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::OutputPort<bool> outPort;

    RTT::InputPort<bool> inPort_1;
    RTT::InputPort<bool> inPort_2;
    RTT::InputPort<bool> inPort_3;
    RTT::InputPort<bool> inPort_4;
};
#endif
