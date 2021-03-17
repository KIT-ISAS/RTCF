#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include <map>
#include <rtt/Activity.hpp>
#include <rtt/DataFlowInterface.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/base/InputPortInterface.hpp>
#include <rtt/base/OutputPortInterface.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <typeinfo>
#include <vector>

#include "main_context.hpp"
#include "rt_runner_types.hpp"
#include "rtcf/rtcf_types.hpp"

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
                                        std::make_pair("no_wait", Mode::NO_WAIT)};
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
    Settings settings_;

    void analyzeDependencies();
    bool generateRTOrder();

    void connectPorts();
    void disconnectPorts();

    void connectOrocosPorts();
    void connectPortsToRos();

    RTT::TaskContext* createInstance(const std::string& component_type, const std::string& component_name);

    void setSlavesOnMainContext();

    bool is_active_ = false;

  public:
    RTRunner();

    void configure(const Settings& settings);
    void shutdown();
    void stopComponents();

    bool loadOrocosComponent(const LoadAttributes& info);
    bool unloadOrocosComponent(const UnloadAttributes& info);

    void activateRTLoop();
    void deactivateRTLoop();

    RTT::Activity* main_activity_;
    ComponentContainerVector component_containers_;
    InternalConnectionsMap internal_connections_;
    ComponentPredecessorMap component_predecessors_;
    ComponentSuccessorMap component_successors_;
    RTOrder rt_order_;

    MainContext main_context_;
};

#endif /* RT_RUNNER_H */
