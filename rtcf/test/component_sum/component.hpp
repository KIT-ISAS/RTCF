#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
OROCOS_HEADERS_END

#include <std_msgs/Float64.h>

class SumTest : public RTT::TaskContext {
  public:
    SumTest(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    RTT::OutputPort<std_msgs::Float64> out_;
    RTT::InputPort<std_msgs::Float64> in1_;
    RTT::InputPort<std_msgs::Float64> in2_;

    std_msgs::Float64 in_msg_1_;
    std_msgs::Float64 in_msg_2_;
    std_msgs::Float64 out_msg_;
};
#endif
