#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include "main_context.hpp"
#include <typeinfo>
#include <vector>

#include "rtt/base/InputPortInterface.hpp"
#include "rtt/base/OutputPortInterface.hpp"
#include "rtt/DataFlowInterface.hpp"

#include "rtt/extras/SlaveActivity.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>


struct OrocosContainer {
    OrocosContainer(RTT::TaskContext* taskContext,
                    RTT::extras::SlaveActivity* activity)
        : taskContext(taskContext), activity(activity) {
        getPorts();
    }

    RTT::TaskContext* taskContext;
    RTT::extras::SlaveActivity* activity;

    RTT::DataFlowInterface::Ports input_ports;
    RTT::DataFlowInterface::Ports output_ports;

    void getPorts() {
        for (RTT::base::PortInterface* port :
             taskContext->ports()->getPorts()) {
            if (typeid(port) == typeid(RTT::base::InputPortInterface)) {
                input_ports.push_back(port);
            } else if (typeid(port) == typeid(RTT::base::OutputPortInterface)) {
                output_ports.push_back(port);
            } else {
                std::cout << "Port is neighter Input or Output Port"
                          << std::endl;
            }
        }
    }
};

class RTRunner {
   private:
    bool infererOrder();

    bool createFromLibrary(std::string componentType, std::string componentName,
                           RTT::TaskContext*& task);

    void setSlavesOnMainContext();


    bool isActive;

   public:
    RTRunner();

    void configure();
    void shutdown();

    bool loadOrocosComponent(std::string componentType,
                             std::string componentName);
    bool unloadOrocosComponent();

    void activateRTLoopCallback();
    void deactivateRTLoopCallback();

    void setFrequency(float frequency);
    void setPeriod(float period);

    RTT::Activity* main_activity_;
    std::vector<OrocosContainer> orocosContainer_;

    float period_;
    MainContext main_context_;
};


#endif /* RT_RUNNER_H */
