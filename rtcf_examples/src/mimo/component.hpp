#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <std_msgs/Float64.h>

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
OROCOS_HEADERS_END

class Mimo : public RTT::TaskContext {
  public:
    Mimo(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    RTT::OutputPort<std_msgs::Float64> port_out1_;
    RTT::OutputPort<std_msgs::Float64> port_out2_;

    RTT::InputPort<std_msgs::Float64> port_in1_;
    RTT::InputPort<std_msgs::Float64> port_in2_;
    RTT::InputPort<std_msgs::Float64> port_in3_;
    RTT::InputPort<std_msgs::Float64> port_in4_;
};
#endif
