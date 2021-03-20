#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf_example_msgs/Custom.h>

// This header is causing many warnings, so we disable them temporarily
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#pragma GCC diagnostic pop

class Mimo : public RTT::TaskContext {
  public:
    Mimo(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    RTT::OutputPort<rtcf_example_msgs::Custom> port_out1_;
    RTT::OutputPort<rtcf_example_msgs::Custom> port_out2_;

    RTT::InputPort<rtcf_example_msgs::Custom> port_in1_;
    RTT::InputPort<rtcf_example_msgs::Custom> port_in2_;
    RTT::InputPort<rtcf_example_msgs::Custom> port_in3_;
    RTT::InputPort<rtcf_example_msgs::Custom> port_in4_;
};
#endif
