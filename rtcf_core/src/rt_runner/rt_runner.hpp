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

    void connectPorts();
    void connectOrocosPorts();
    void connectPortsToRos();
    void disconnectAllPorts();


    GraphOrocosContainers buildGraph();

    bool createFromLibrary(std::string componentType, std::string componentName,
                           RTT::TaskContext*& task);

    void setSlavesOnMainContext();


    bool isActive = false;

   public:
    RTRunner();

    void configure();
    void shutdown();
    void stopComponents();

    bool loadOrocosComponent(std::string componentType,
                             std::string componentName, std::string ns,
                             std::string topics_ignore_for_graph, bool is_start,
                             bool is_sync, std::vector<mapping> mappings);
    bool unloadOrocosComponent(std::string componentName, std::string ns);

    void activateRTLoop();
    void deactivateRTLoop();

    void setFrequency(float frequency);
    void setPeriod(float period);
    void setMode(std::string mode);
    void setNumComponentsExpected(int num);
    void setWhitelistRosMapping(std::string whitelist);
    void setTopicsIgnoreForGraph(std::string topics_ignore_for_graph);


    RTT::Activity* main_activity_;
    std::vector<OrocosContainer> orocosContainer_;

    GraphOrocosContainers RTOrder;
    GraphOrocosContainers active_graph_;

    float period_ = 1.0;
    MainContext main_context_;

    std::string mode_ = "inactive";
    int num_components_expected_ = 0;
    std::string whitelist_ros_mapping_ = "";
    std::string topics_ignore_for_graph_ = "";
};


#endif /* RT_RUNNER_H */
