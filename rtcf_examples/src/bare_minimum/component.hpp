#ifndef COMPONENT_HPP
#define COMPONENT_HPP

#include <rtt/RTT.hpp>

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
