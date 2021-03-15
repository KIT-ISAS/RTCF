#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include <map>
#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>
#include <typeinfo>
#include <vector>

#include "main_context.hpp"
#include "rtcf_types.hpp"
#include "rtt/DataFlowInterface.hpp"
#include "rtt/base/InputPortInterface.hpp"
#include "rtt/base/OutputPortInterface.hpp"
#include "rtt/extras/SlaveActivity.hpp"

class RTRunner {
  public:
    enum class Mode {
        WAIT_FOR_COMPONENTS,
        WAIT_FOR_TRIGGER,
        NO_WAIT,
        UNKNOWN,
    };
    static Mode string2Mode(const std::string& s) {
        // clang-format off
        std::map<std::string, Mode> map{std::make_pair("wait_for_components", Mode::WAIT_FOR_COMPONENTS),
                                        std::make_pair("wait_for_trigger", Mode::WAIT_FOR_TRIGGER),
                                        std::make_pair("no_wait", Mode::NO_WAIT), 
                                        std::make_pair("", Mode::UNKNOWN)};
        // clang-format on
        const auto it = map.find(s);
        if (it == map.end()) {
            return RTRunner::Mode::UNKNOWN;
        }
        return map[s];
    }

    struct Settings {
        Mode mode;
        size_t expected_num_components;
        std::string ros_mapping_whitelist;
        std::string ros_mapping_blacklist;
        double frequency;
    };

  private:
    Settings settings;

    void generateRTOrder();

    void connectPorts();
    void connectOrocosPorts();
    void connectPortsToRos();
    void disconnectAllPorts();

    GraphOrocosContainers buildGraph();

    bool createFromLibrary(std::string componentType, std::string componentName, RTT::TaskContext*& task);

    void setSlavesOnMainContext();

    bool isActive = false;

  public:
    RTRunner();

    void configure(Settings settings);
    void shutdown();
    void stopComponents();

    bool loadOrocosComponent(std::string componentType, std::string componentName, std::string ns,
                             std::string topics_ignore_for_graph, bool is_first, bool is_sync,
                             std::vector<mapping> mappings);
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

    std::string mode_                    = "inactive";
    int num_components_expected_         = 0;
    std::string whitelist_ros_mapping_   = "";
    std::string topics_ignore_for_graph_ = "";
};

#endif /* RT_RUNNER_H */
