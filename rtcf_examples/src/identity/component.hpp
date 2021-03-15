#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf_example_msgs/Custom.h>

#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

class Identity : public RTT::TaskContext {
  public:
    Identity(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
    RTT::OutputPort<rtcf_example_msgs::Custom> port_out_;
    RTT::InputPort<rtcf_example_msgs::Custom> port_in_;
    rtcf_example_msgs::Custom msg;
};
#endif
