#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/RTT.hpp>
OROCOS_HEADERS_END

#include <rtcf/rtcf_extension.hpp>

class Logging : public RTT::TaskContext, public RtcfExtension {
  public:
    Logging(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};

#endif
