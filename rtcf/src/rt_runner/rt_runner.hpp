#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include <atomic>
#include <regex>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rtt/TaskContext.hpp>
#pragma GCC diagnostic pop

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
        bool is_simulation;
    };

  private:
    RTT::base::ActivityInterface* createMainActivity();

    void activateRTLoop();
    void deactivateRTLoop();

    void tryStartExecution();
    void stopExecution();

    void analyzeDependencies();
    bool generateRTOrder();

    void connectPorts();
    void disconnectPorts();

    void connectOrocosPorts();
    void connectPortsToRos();

    RTT::TaskContext* createInstance(const std::string& component_type, const std::string& component_name);

    void setSlavesOnMainContext();

    Settings settings_;
    bool is_active_external_;
    bool is_shutdown_;

    MainContext main_context_;
    RTOrder rt_order_;

    std::regex whitelist_;
    std::regex blacklist_;

    ComponentContainerVector component_containers_;

    InternalConnectionsMap internal_connections_;
    ComponentPredecessorMap component_predecessors_;
    ComponentSuccessorMap component_successors_;

    std::atomic<size_t> num_loaded_components_;

  public:
    RTRunner();

    void configure(const Settings& settings);
    void shutdown();

    bool loadOrocosComponent(const LoadAttributes& info);
    bool unloadOrocosComponent(const UnloadAttributes& info);

    size_t getNumLoadedComponents();  // this method is thread-safe

    // these methods are only available in WAIT_FOR_TRIGGER-mode
    void activateTrigger();
    void deactivateTrigger();
};

#endif /* RT_RUNNER_H */
