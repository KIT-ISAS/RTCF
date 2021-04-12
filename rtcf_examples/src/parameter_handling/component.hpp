#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf_examples/ExampleConfig.h>

#include <rtcf/rt_dynamic_reconfigure.hpp>
#include <rtcf/rtcf_extension.hpp>
#include <rtt/RTT.hpp>

class ParameterHandling : public RTT::TaskContext, public RtcfExtension {
  public:
    ParameterHandling(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTDynamicReconfigure<rtcf_examples::ExampleConfig> dynamic_config;
};
#endif
