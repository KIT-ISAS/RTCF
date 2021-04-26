#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include <atomic>
#include <regex>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/TaskContext.hpp>
OROCOS_HEADERS_END

#include <rtcf_msgs/ComponentInfo.h>
#include <rtcf_msgs/ConnectionInfo.h>
#include <rtcf_msgs/PortInfo.h>

#include "main_context.hpp"
#include "prefaulting.hpp"
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

    enum class WaitPolicy {
        ABOLSUTE = ORO_WAIT_ABS,
        RELATIVE = ORO_WAIT_REL,
        UNKNOWN,
    };
    static WaitPolicy string2WaitPolicy(const std::string& s) {
        // clang-format off
        std::map<std::string, WaitPolicy> map{std::make_pair("absolute", WaitPolicy::ABOLSUTE),
                                        std::make_pair("relative", WaitPolicy::RELATIVE)};
        // clang-format on
        const auto it = map.find(s);
        if (it == map.end()) {
            return RTRunner::WaitPolicy::UNKNOWN;
        }
        return map[s];
    }

    struct Settings {
        Mode mode;
        WaitPolicy wait_policy;
        size_t expected_num_components;
        std::string ros_mapping_whitelist;
        std::string ros_mapping_blacklist;
        double frequency;
        bool is_simulation;
        bool lock_memory;
        size_t safe_heap_size;
        size_t safe_stack_size;
        unsigned cpu_affinity;
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
    PrefaultingExecutable prefaulting_executable_;
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
    void finalize();

    bool loadOrocosComponent(const LoadAttributes& info);
    bool unloadOrocosComponent(const UnloadAttributes& info);

    size_t getNumLoadedComponents();  // this method is thread-safe

    // these methods are only available in WAIT_FOR_TRIGGER-mode
    void activateTrigger();
    void deactivateTrigger();

    // for introspection
    std::vector<rtcf_msgs::ComponentInfo> getComponentInfos();
    std::vector<rtcf_msgs::ConnectionInfo> getConnectionInfos();
    std::vector<std::string> getComponentOrder();
};

#endif /* RT_RUNNER_H */
