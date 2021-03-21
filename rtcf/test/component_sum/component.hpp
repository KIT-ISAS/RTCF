#ifndef COMPONENT_HPP
#define COMPONENT_HPP


// This header is causing many warnings, so we disable them temporarily
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#pragma GCC diagnostic pop

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
