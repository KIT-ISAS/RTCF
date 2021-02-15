#ifndef OROCOS_SUM_COMPONENT_HPP
#define OROCOS_SUM_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>

#include <rtcf/rtcf_extension.hpp>

class Sum : public RTT::TaskContext, public RtcfExtension {
   public:
    Sum(std::string const& name);
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

    std_msgs::Float64 in_msg_1;
    std_msgs::Float64 in_msg_2;
    std_msgs::Float64 in_msg_3;
    std_msgs::Float64 in_msg_4;

    std_msgs::Float64 out_msg;

    bool new_msg_1;
    bool new_msg_2;
    bool new_msg_3;
    bool new_msg_4;
};
#endif
