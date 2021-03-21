#ifndef COMPONENT_HPP
#define COMPONENT_HPP

// This header is causing many warnings, so we disable them temporarily
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
#pragma GCC diagnostic pop

#include <std_msgs/Float64.h>

#include <rtcf/rtcf_extension.hpp>

class ExtensionTest : public RTT::TaskContext, public RtcfExtension {
  public:
    ExtensionTest(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

  private:
};
#endif
