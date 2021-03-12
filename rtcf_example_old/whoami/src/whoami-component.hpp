#ifndef OROCOS_WHOAMI_COMPONENT_HPP
#define OROCOS_WHOAMI_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <rtcf/rtcf_extension.hpp>

class Whoami : public RTT::TaskContext, public RtcfExtension {
  public:
    Whoami(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::OutputPort<std_msgs::Float64> outPort;

    RTT::InputPort<std_msgs::Float64> inPort_1;
    RTT::InputPort<std_msgs::Float64> inPort_2;
    RTT::InputPort<std_msgs::Float64> inPort_3;
    RTT::InputPort<std_msgs::Float64> inPort_4;
};
#endif
