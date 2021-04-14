#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
OROCOS_HEADERS_END

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
