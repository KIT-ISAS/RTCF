#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/RTT.hpp>
OROCOS_HEADERS_END

class BareMinimum : public RTT::TaskContext {
  public:
    BareMinimum(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
};
#endif
