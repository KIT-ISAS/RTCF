#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf/rtcf_extension.hpp>
#include <rtt/RTT.hpp>

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
