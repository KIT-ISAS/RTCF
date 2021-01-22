#ifndef MAIN_CONTEXT_H
#define MAIN_CONTEXT_H

#include <rtt/RTT.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>
#include <vector>

class MainContext : public RTT::TaskContext
{
private:
    

public:
    MainContext(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setSlaves(std::vector<RTT::extras::SlaveActivity*>);
    void clearSlaves();

    std::vector<RTT::extras::SlaveActivity*> slaves_;
};

#endif /* MAIN_CONTEXT_H */
