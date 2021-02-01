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

struct mapping {
    std::string from_topic;
    std::string to_topic;
};

struct OrocosContainer {
    OrocosContainer(std::string componentType, std::string componentName,
                    bool is_start, std::vector<mapping> mappings,
                    RTT::TaskContext* taskContext,
                    RTT::extras::SlaveActivity* activity)
        : componentType_(componentType),
          componentName_(componentName),
          is_start_(is_start),
          mappings_(mappings),
          taskContext_(taskContext),
          activity_(activity) {
        getPorts();
    }

    std::string componentType_;
    std::string componentName_;
    bool is_start_;
    std::vector<mapping> mappings_;

    RTT::TaskContext* taskContext_;
    RTT::extras::SlaveActivity* activity_;

    RTT::DataFlowInterface::Ports input_ports_;
    RTT::DataFlowInterface::Ports output_ports_;

    void getPorts() {
        for (RTT::base::PortInterface* port :
             taskContext_->ports()->getPorts()) {
            if (typeid(port) == typeid(RTT::base::InputPortInterface)) {
                input_ports_.push_back(port);
            } else if (typeid(port) == typeid(RTT::base::OutputPortInterface)) {
                output_ports_.push_back(port);
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
    void buildRTOrder();


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
