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

  private:
    // This class is necessary overhead for using dynamic reconfigure
    // The reason therefore is that, the package::NameConfig-struct does always contain a string, which is definitely
    // not real-time safe. Thus, we do this in the non real-time part.
    struct ExampleParameters {
        using ConfigType = rtcf_examples::ExampleConfig;
        // list all fields of the .cfg-file
        int int_param;
        double double_param;
        bool bool_param;
        int size;

        ExampleParameters(const ConfigType& config) :
            int_param(config.int_param),
            double_param(config.double_param),
            bool_param(config.bool_param),
            size(config.size) {}
        ExampleParameters(){}
    };
    // this prepares the dynamic reconfigure server
    RTDynamicReconfigure<ExampleParameters> dynamic_config;
};
#endif
