#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>

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
    void activateRTLoop();
    void deactivateRTLoop();

    void analyzeDependencies();
    bool generateRTOrder();

    void connectPorts();
    void disconnectPorts();

    void connectOrocosPorts();
    void connectPortsToRos();

    RTT::TaskContext* createInstance(const std::string& component_type, const std::string& component_name);

    void setSlavesOnMainContext();

    Settings settings_;
    bool is_active_ = false;

    RTT::Activity* main_activity_;
    MainContext main_context_;
    RTOrder rt_order_;

    ComponentContainerVector component_containers_;

    InternalConnectionsMap internal_connections_;
    ComponentPredecessorMap component_predecessors_;
    ComponentSuccessorMap component_successors_;

  public:
    RTRunner();

    void configure(const Settings& settings);
    void shutdown();        // TODO: clean
    void stopComponents();  // TODO: clean

    bool loadOrocosComponent(const LoadAttributes& info);
    bool unloadOrocosComponent(const UnloadAttributes& info);

    // these methods are only available in WAIT_FOR_TRIGGER-mode
    void activateTrigger();
    void deactivateTrigger();
};

#endif /* RT_RUNNER_H */
