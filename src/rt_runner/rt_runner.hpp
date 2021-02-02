#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include "main_context.hpp"
#include "rtcf_types.hpp"
#include <typeinfo>
#include <vector>

#include "rtt/base/InputPortInterface.hpp"
#include "rtt/base/OutputPortInterface.hpp"
#include "rtt/DataFlowInterface.hpp"

#include "rtt/extras/SlaveActivity.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

class RTRunner {
   private:
    void generateRTOrder();
    GraphOrocosContainers buildGraph();

    bool createFromLibrary(std::string componentType, std::string componentName,
                           RTT::TaskContext*& task);

    void setSlavesOnMainContext();


    bool isActive;

   public:
    RTRunner();

    void configure();
    void shutdown();

    bool loadOrocosComponent(std::string componentType,
                             std::string componentName, bool is_start,
                             std::vector<mapping> mappings);
    bool unloadOrocosComponent(std::string componentName);

    void activateRTLoop();
    void deactivateRTLoop();

    void setFrequency(float frequency);
    void setPeriod(float period);

    RTT::Activity* main_activity_;
    std::vector<OrocosContainer> orocosContainer_;

    float period_;
    MainContext main_context_;
};


#endif /* RT_RUNNER_H */
