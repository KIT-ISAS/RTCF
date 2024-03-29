#include "rt_runner.hpp"

#include <ros/ros.h>

#include "rtcf/macros.hpp"
#include "rtcf/rt_rosconsole_logging.hpp"
#include "rtcf/rtcf_extension.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt_ros/rtt_ros.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_thread.h>
#include <rtt_roscomm/rostopic.h>

#include <rtt/base/ActivityInterface.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/extras/PeriodicActivity.hpp>
#include <rtt/extras/SlaveActivity.hpp>
OROCOS_HEADERS_END

RTRunner::RTRunner() :
    is_active_external_(false), is_shutdown_(false), main_context_("main_context"), num_loaded_components_(0) {}

void RTRunner::configure(const Settings& settings) {
    settings_                 = settings;
    RtcfExtension::frequency_ = settings_.frequency;
    // period is made available thorugh TaskContext::getPeriod()
    // RtcfExtension::period_    = 1.0 / settings_.frequency;

    // configure logging (before any spinner is started to not miss any service callbacks)
    RtRosconsoleLogging::getInstance().configure();
    RtRosconsoleLogging::getInstance().start();

    // create and configure the main worker thread
    RTT::base::ActivityInterface* main_activity = createMainActivity();

    // add thread to context
    main_context_.setActivity(main_activity);
    // according to OROCOS doc, main activity is now owned by the main_context and shall only be reference through
    // getActivity().
    main_context_.configure();

    // do further thread configuration
    if (settings_.lock_memory) {
        // avoid page faults
        prefaulting_executable_ = PrefaultingExecutable(settings_.safe_heap_size, settings_.safe_stack_size);
        main_context_.engine()->runFunction(&prefaulting_executable_);
    }

    // whitelist/blacklist exceptions
    try {
        whitelist_ = std::regex(settings_.ros_mapping_whitelist);
    } catch (std::regex_error e) {
        ROS_ERROR_STREAM("Invalid regex in ros_mapping_whitelist. Falling back to default '.*'.");
        whitelist_ = std::regex(".*");
    }
    try {
        blacklist_ = std::regex(settings_.ros_mapping_blacklist);
    } catch (std::regex_error e) {
        ROS_ERROR_STREAM("Invalid regex in ros_mapping_blacklist. Falling back to default ''.");
        blacklist_ = std::regex("");
    }
};

RTT::base::ActivityInterface* RTRunner::createMainActivity() {
    RTT::base::ActivityInterface* main_activity;
    if (!settings_.is_simulation) {
        // configure thread for realtimeness
        RTT::extras::PeriodicActivity* activity =
            new RTT::extras::PeriodicActivity(ORO_SCHED_RT, 98, 1.0 / settings_.frequency);
        activity->thread()->setWaitPeriodPolicy(static_cast<int>(settings_.wait_policy));
        main_activity = activity;
    } else {
        ROS_WARN("RTCF is in simulation mode! Neither real-timeness nor set frequencies are guaranteed.");
        // this activity will be triggered according to the clock signals
        // NOTE: clock accuracy will depend on the simulation clock source
        main_activity = new rtt_rosclock::SimClockActivity(1.0 / settings_.frequency);
        rtt_rosclock::use_ros_clock_topic();
        // NOTE: setting the sim-clock-thread to real-time priority is not really beneficial
        // as the clock input is not real-time anyway
        // rtt_rosclock::SimClockThread::Instance()->setScheduler(ORO_SHED_RT);
        // rtt_rosclock::SimClockThread::Instance()->setPriority(98);
        rtt_rosclock::enable_sim();
    }

    // set CPU affinity
    main_activity->setCpuAffinity(settings_.cpu_affinity);

    return main_activity;
}

void RTRunner::shutdown() {
    is_shutdown_ = true;
    stopExecution();
}

void RTRunner::finalize() {
    main_context_.cleanup();

    // stop the rosclock thread if it exists
    if (rtt_rosclock::SimClockThread::GetInstance()) {
        rtt_rosclock::SimClockThread::GetInstance()->stop();
    }

    // stop the logging
    RtRosconsoleLogging::getInstance().stop();
    RtRosconsoleLogging::getInstance().cleanup();
}

size_t RTRunner::getNumLoadedComponents() { return num_loaded_components_; }

void RTRunner::activateTrigger() {
    if (is_shutdown_) {
        return;
    }
    assert(settings_.mode == Mode::WAIT_FOR_TRIGGER);
    if (!is_active_external_) {
        is_active_external_ = true;
        tryStartExecution();
    }
}

void RTRunner::deactivateTrigger() {
    if (is_shutdown_) {
        return;
    }
    assert(settings_.mode == Mode::WAIT_FOR_TRIGGER);
    if (is_active_external_) {
        is_active_external_ = false;
        stopExecution();
    }
}

bool RTRunner::loadOrocosComponent(const LoadAttributes& info) {
    if (is_shutdown_) {
        return false;
    }
    if (settings_.mode == Mode::WAIT_FOR_COMPONENTS &&
        component_containers_.size() >= settings_.expected_num_components) {
        ROS_ERROR_STREAM("More components were loaded than expected. Additional components will not be active.");
        return false;
    }

    // load package
    if (!rtt_ros::import(info.rt_package)) {
        ROS_ERROR_STREAM("Could not load ROS package " << info.rt_package << " into OROCOS.");
        return false;
    }
    // create component instance
    RTT::TaskContext* task = createInstance(info.rt_type, info.name);
    if (!task) {
        ROS_ERROR_STREAM("Could not instantiate component of type " << info.rt_type << " with name " << info.name);
        return false;
    }

    // prepare sequential execution
    RTT::extras::SlaveActivity* slave_activity = new RTT::extras::SlaveActivity();
    task->setActivity(slave_activity);
    // try to configure the component
    ComponentContainer component_container(info, task);
    if (!task->configure()) {
        ROS_ERROR_STREAM("configuration() call to component " << info.name << " failed");
        delete task;
        return false;
    }
    component_container.handlePostConfiguration();

    // task->start() should not be called here as this would trigger the updateHook() prematurely

    // stop the execution now (if not already stopped)
    // it is not necessary to stop the loop earlier because the component does not get triggered
    stopExecution();
    component_containers_.push_back(component_container);
    num_loaded_components_ = component_containers_.size();

    tryStartExecution();

    return true;
}

RTT::TaskContext* RTRunner::createInstance(const std::string& component_type, const std::string& component_name) {
    RTT::TaskContext* tc = nullptr;
    tc                   = RTT::ComponentLoader::Instance()->loadComponent(component_name, component_type);
    return tc;
}

bool RTRunner::unloadOrocosComponent(const UnloadAttributes& info) {
    // find the component to remove
    auto result = std::find_if(component_containers_.begin(), component_containers_.end(),
                               [&info](const auto& container) { return container.attributes.name == info.name; });
    if (result == component_containers_.end()) {
        return false;
    }

    stopExecution();

    // tear down the component
    RTT::TaskContext* task = (*result).task_context;
    task->cleanup();
    delete task;
    component_containers_.erase(result);
    num_loaded_components_ = component_containers_.size();

    tryStartExecution();

    // this always succeeds, if the component the remove was found
    return true;
}

void RTRunner::activateRTLoop() {
    if (!main_context_.isRunning()) {
        // first start all the components that are not already started
        for (const auto& component : component_containers_) {
            auto* tc = component.task_context;
            if (tc->isConfigured() && !tc->isRunning()) {
                if (!tc->start()) {
                    ROS_ERROR_STREAM("start() call to component " << component.attributes.name << " failed.");
                }
            }
        }
        // then go to cyclic operation
        main_context_.start();
    }
}

void RTRunner::deactivateRTLoop() {
    if (main_context_.isRunning()) {
        // first stop all components that are running
        for (const auto& component : component_containers_) {
            auto* tc = component.task_context;
            if (tc->isRunning()) {
                tc->stop();
            }
        }
        // stop cyclic operation
        main_context_.stop();
    }
}

void RTRunner::tryStartExecution() {
    if (main_context_.isRunning()) {
        // if main context is running, there is nothing to do
        return;
    }
    if (is_shutdown_) {
        return;
    }
    // if allowed, (re)start automatically
    if ((settings_.mode == Mode::NO_WAIT && !component_containers_.empty()) ||
        (settings_.mode == Mode::WAIT_FOR_COMPONENTS &&
         component_containers_.size() == settings_.expected_num_components) ||
        (settings_.mode == Mode::WAIT_FOR_TRIGGER && is_active_external_)) {
        analyzeDependencies();
        connectPorts();
        generateRTOrder();
        setSlavesOnMainContext();
        activateRTLoop();
    }
}

void RTRunner::stopExecution() {
    if (!main_context_.isRunning()) {
        // if main context is not running, there is nothing to do
        return;
    }
    deactivateRTLoop();
    disconnectPorts();
    rt_order_.clear();
}

void RTRunner::setSlavesOnMainContext() {
    // NOTE: main context needs to be stopped when calling this
    main_context_.setSlaves(rt_order_);
}

void RTRunner::analyzeDependencies() {
    // discard previously constructed stuff
    internal_connections_.clear();
    component_predecessors_.clear();
    component_successors_.clear();

    // sort all components according to name to ensure determinism
    std::sort(component_containers_.begin(), component_containers_.end(),
              [](const auto& a, const auto& b) { return (a.attributes.name < b.attributes.name); });

    // iterate over all output ports
    for (const auto& c_from : component_containers_) {
        for (const auto& p_output : c_from.output_ports) {
            // iterate over all input ports
            for (const auto& c_to : component_containers_) {
                for (const auto& p_input : c_to.input_ports) {
                    // check if the ports are connected
                    if (p_output.mapped_name == p_input.mapped_name) {
                        // check whether the current input is supplied by a single source
                        if (internal_connections_.find(&p_input) != internal_connections_.end()) {
                            ROS_WARN_STREAM("Found multiple connections to input port "
                                            << p_input.mapped_name << " of component " << c_to.attributes.name << ". "
                                            << "Therefore, this connection will be ignored. Check your launchfile!");
                            continue;
                        }
                        // extract relevant information
                        // 1. input ports that are supplied by an output port (aka internal connections)
                        internal_connections_[&p_input] = &p_output;
                        ROS_DEBUG_STREAM("Connected " << c_from.attributes.name << " (" << p_output.original_name
                                                      << ") with " << c_to.attributes.name << " ("
                                                      << p_input.original_name << ") via " << p_input.mapped_name);
                        // exclude the ignored topics here or components with is_first attribute set
                        if (!std::regex_match(p_input.mapped_name, c_to.topics_ignore_regex) &&
                            !c_to.attributes.is_first) {
                            // 2. predecessor components (aka input dependencies)
                            if (component_predecessors_[&c_to].insert(&c_from).second) {
                                ROS_DEBUG_STREAM("Added " << c_from.attributes.name
                                                          << " as predecessor (dependency) for " << c_to.attributes.name
                                                          << ".");
                            }
                            // 3. successor components (aka output dependencies)
                            if (component_successors_[&c_from].insert(&c_to).second) {
                                ROS_DEBUG_STREAM("Added " << c_to.attributes.name << " as successor for "
                                                          << c_from.attributes.name << ".");
                            }
                        } else {
                            ROS_DEBUG_STREAM("Ignored " << c_from.attributes.name << " as predecessor (dependency) for "
                                                        << c_to.attributes.name << " due to explicit exclusion.");
                        }
                    }
                }
            }
        }
    }
}

bool RTRunner::generateRTOrder() {
    rt_order_.clear();

    // We have a dependency graph defined by component_predecessors_ and component_successors_.
    // So we use Kahn's algorithm to determine a topological order.
    // https://en.wikipedia.org/wiki/Topological_sorting#Kahn's_algorithm

    RTOrder L;                              // sorted elements
    std::set<const ComponentContainer*> S;  // nodes with no incoming edge
    // find all nodes with no incoming edge (predecessors empty or is_first set)
    for (const auto& component : component_containers_) {
        if (component_predecessors_[&component].empty()) {
            S.insert(&component);
        }
    }

    while (!S.empty()) {
        // transfer first element from S
        const auto* n = *(S.begin());
        S.erase(S.begin());
        L.push_back(n);
        for (const auto& m : component_successors_[n]) {
            // remove satisfied dependency n from component m
            component_predecessors_[m].erase(n);
            if (component_predecessors_[m].empty()) {
                S.insert(m);
            }
        }
    }

    // check if all dependencies are met (maybe more checks than necessary ;-))
    bool valid_solution = true;
    // 1. L must have only unique elements
    valid_solution &= (std::set<const ComponentContainer*>(L.begin(), L.end()).size() == L.size());
    // 2. L must be same size as component_containers_
    valid_solution &= (L.size() == component_containers_.size());
    // 3. dependencies must be empty
    for (const auto& component : component_containers_) {
        valid_solution &= component_predecessors_[&component].empty();
    }
    if (!valid_solution) {
        // error information for debugging
        ROS_ERROR_STREAM("Automatic dependency resolution failed. This might originate from cyclic dependencies. "
                         << "Check is_first and topics_ignore_for_graph parameters in launchfile. "
                         << "Here is some debug information that might help:");

        std::stringstream ss;
        ss << "Loaded components: ";
        for (const auto& c : component_containers_) {
            ss << c.attributes.name << ", ";
        }
        ROS_ERROR_STREAM(ss.str());

        ss.clear();
        ss << "Determined order: ";
        for (const auto& item : L) {
            ss << item->attributes.name << ", ";
        }
        ROS_ERROR_STREAM(ss.str());

        ss.clear();
        ss << "Following dependencies remained: " << std::endl;
        for (const auto& component : component_containers_) {
            if (!component_predecessors_[&component].empty()) {
                for (const auto& predecessor : component_predecessors_[&component]) {
                    ss << "Component " << component.attributes.name << " depends on " << predecessor->attributes.name
                       << std::endl;
                }
            }
        }
        ROS_ERROR_STREAM(ss.str());

        return false;
    } else {
        rt_order_ = L;
        // debug print of RT order
        std::stringstream ss;
        ss << "RT order is determined as: ";
        for (const auto& item : rt_order_) {
            ss << item->attributes.name << ", ";
        }
        ROS_INFO_STREAM(ss.str());
        return true;
    }
}

void RTRunner::connectPorts() {
    connectOrocosPorts();
    connectPortsToRos();
}

void RTRunner::disconnectPorts() {
    // simply disconnect all input and output ports
    // this also stops ROS connections created by createStream()
    for (auto& container : component_containers_) {
        for (auto& output_port : container.output_ports) {
            output_port.port->disconnect();
            output_port.connected_to_ROS = false;
        }
        for (auto& input_port : container.input_ports) {
            input_port.port->disconnect();
            input_port.connected_to_ROS = false;
        }
    }

    internal_connections_.clear();
}

void RTRunner::connectOrocosPorts() {
    for (const auto& connection : internal_connections_) {
        const auto& to_port   = connection.first;
        const auto& from_port = connection.second;
        // the default connection policy does not use a buffer, which is what we want
        to_port->port->connectTo(from_port->port);
    }
}

void RTRunner::connectPortsToRos() {
    auto check_name = [this](const std::string& name) {
        // elements that match whitelist
        if (std::regex_match(name, whitelist_)) {
            // but not blacklist are passed through
            // (empty blacklist means that it never matches)
            if (!std::regex_match(name, blacklist_)) {
                return true;
            }
        }
        return false;
    };

    // all output ports that meet whitelist/blacklist are always linked to ROS
    for (auto& container : component_containers_) {
        for (auto& output_port : container.output_ports) {
            if (check_name(output_port.mapped_name)) {
                output_port.port->createStream(rtt_roscomm::topic(output_port.mapped_name));
                output_port.connected_to_ROS = true;
                ROS_DEBUG_STREAM("Connected " << container.attributes.name << " (output " << output_port.original_name
                                              << ") to ROS topic " << output_port.mapped_name << "");
            }
        }
    }
    // all input ports that meet whitelist/blacklist
    // AND that do not have an internal connection to an output port are linked to ROS
    // this is required to avoid side-channels through ROS
    for (auto& container : component_containers_) {
        for (auto& input_port : container.input_ports) {
            if (check_name(input_port.mapped_name)) {
                // check for internal connection
                if (internal_connections_.count(&input_port) == 0) {
                    input_port.port->createStream(rtt_roscomm::topic(input_port.mapped_name));
                    input_port.connected_to_ROS = true;
                    ROS_DEBUG_STREAM("Connected " << container.attributes.name << " (input " << input_port.original_name
                                                  << ") to ROS topic " << input_port.mapped_name << "");
                } else {
                    ROS_DEBUG_STREAM("Skipped connecting " << container.attributes.name << " (input "
                                                           << input_port.original_name << ") to ROS topic "
                                                           << input_port.mapped_name << " due to internal connection");
                }
            }
        }
    }
}

std::vector<rtcf_msgs::ComponentInfo> RTRunner::getComponentInfos() {
    std::vector<rtcf_msgs::ComponentInfo> all_infos;

    for (const auto& component : component_containers_) {
        rtcf_msgs::ComponentInfo component_info;
        component_info.name    = component.attributes.name;
        component_info.package = component.attributes.rt_package;
        component_info.type    = component.attributes.rt_type;

        auto create_port_info = [this](const PortContainer& port, rtcf_msgs::PortInfo::_port_type_type port_type) {
            rtcf_msgs::PortInfo port_info;
            port_info.port_type        = port_type;
            port_info.name_remapped    = port.mapped_name;
            port_info.name_unresolved  = port.original_name;
            port_info.msg_type         = port.message_type;
            port_info.connected_to_ROS = port.connected_to_ROS;
            return port_info;
        };

        for (const auto& in_port : component.input_ports) {
            component_info.input_ports.push_back(create_port_info(in_port, rtcf_msgs::PortInfo::TYPE_INPUT));
        }
        for (const auto& out_port : component.output_ports) {
            component_info.output_ports.push_back(create_port_info(out_port, rtcf_msgs::PortInfo::TYPE_OUTPUT));
        }

        all_infos.push_back(component_info);
    }
    return all_infos;
}

std::vector<rtcf_msgs::ConnectionInfo> RTRunner::getConnectionInfos() {
    // we use a map for collecting our data in a more efficient way
    struct ConnectionData {
        std::string type                             = "";
        std::set<std::string> publishing_components  = {};
        std::set<std::string> subscribing_components = {};
        bool connected_to_ROS                        = false;
    };
    std::map<std::string, ConnectionData> data;

    // populate the map
    for (const auto& connection : internal_connections_) {
        const auto& port_to   = *connection.first;
        const auto& port_from = *connection.second;

        ConnectionData& current_connection = data[port_from.mapped_name];

        // fill in data from ports
        // message type
        if (current_connection.type == "") {
            current_connection.type = port_from.message_type;
        } else {
            // must be the same if already set
            assert(current_connection.type == port_from.message_type);
        }
        // ROS connection
        current_connection.connected_to_ROS |= port_from.connected_to_ROS;
        current_connection.connected_to_ROS |= port_to.connected_to_ROS;
        // if both, input and output port are connected to ROS, a non-real-time bypass is created
        assert(!(port_to.connected_to_ROS && port_from.connected_to_ROS));
        // publishers and subscribers
        current_connection.publishing_components.insert(port_from.component_name);
        current_connection.subscribing_components.insert(port_to.component_name);
    }

    // then convert the map to ROS message format
    std::vector<rtcf_msgs::ConnectionInfo> all_infos;
    for (const auto& entry : data) {
        rtcf_msgs::ConnectionInfo conn_info;
        conn_info.name             = entry.first;
        conn_info.type             = entry.second.type;
        conn_info.connected_to_ROS = entry.second.connected_to_ROS;
        std::copy(entry.second.publishing_components.begin(), entry.second.publishing_components.end(),
                  std::back_inserter(conn_info.publishing_components));
        std::copy(entry.second.subscribing_components.begin(), entry.second.subscribing_components.end(),
                  std::back_inserter(conn_info.subscribing_components));
        all_infos.push_back(conn_info);
    }

    return all_infos;
}

std::vector<std::string> RTRunner::getComponentOrder() {
    std::vector<std::string> order_info;
    for (const auto& order_element : rt_order_) {
        order_info.push_back(order_element->attributes.name);
    }
    return order_info;
}
