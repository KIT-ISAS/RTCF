#include "rt_runner.hpp"

#include <ros/ros.h>
#include <rtt_ros/rtt_ros.h>
#include <rtt_roscomm/rostopic.h>

#include <iostream>
#include <regex>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <vector>

RTRunner::RTRunner() : is_active_(false), main_context_("main_context"){};

void RTRunner::configure(const Settings& settings) {
    settings_ = settings;

    // TODO: add CPU affinity and similar stuff here
    main_activity_ = new RTT::Activity(ORO_SCHED_RT, 98);
    main_context_.setActivity(main_activity_);
    main_context_.configure();
};
void RTRunner::shutdown() { deactivateRTLoop(); };

void RTRunner::activateRTLoop() {
    if (!is_active_) {
        main_activity_->setPeriod(1.0 / settings_.frequency);
        main_context_.start();
    }
    is_active_ = true;
};

void RTRunner::deactivateRTLoop() {
    if (is_active_) {
        main_context_.stop();
    }
    is_active_ = false;
};

bool RTRunner::loadOrocosComponent(const LoadAttributes& info) {
    if (settings_.mode == Mode::WAIT_FOR_COMPONENTS && rt_order_.size() >= settings_.expected_num_components) {
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
    RTT::extras::SlaveActivity* slave_activity = new RTT::extras::SlaveActivity(main_activity_);
    task->setActivity(slave_activity);
    ComponentContainer component_container(info, task, slave_activity);

    // try to configure the component
    if (!component_container.task_context->configure()) {
        ROS_ERROR_STREAM("configuration() call to component " << component_container.attributes.name << " failed");
        return false;
    }
    // component_container.task_context->start() should not be called here as this would trigger the updateHook()

    // stop the execution now (if not already stopped)
    // it is not necessary to stop the loop earlier because the component does not get triggered
    deactivateRTLoop();
    component_containers_.push_back(component_container);

    // do the magic (connecting nodes, graph resolution, etc.)
    // TODO: only call this when necessary, since this is quite expensive
    disconnectPorts();
    analyzeDependencies();
    connectPorts();
    generateRTOrder();
    setSlavesOnMainContext();

    if ((settings_.mode == Mode::NO_WAIT) ||
        (settings_.mode == Mode::WAIT_FOR_COMPONENTS && rt_order_.size() == settings_.expected_num_components)) {
        activateRTLoop();
    }

    return true;
};

RTT::TaskContext* RTRunner::createInstance(const std::string& component_type, const std::string& component_name) {
    RTT::TaskContext* tc = nullptr;
    tc                   = RTT::ComponentLoader::Instance()->loadComponent(component_name, component_type);
    return tc;
}

bool RTRunner::unloadOrocosComponent(const UnloadAttributes& info) {
    deactivateRTLoop();

    // stop all connections
    disconnectPorts();

    // find the component to remove
    auto result = std::find_if(component_containers_.begin(), component_containers_.end(),
                               [&info](const auto& container) { return container.attributes.name == info.name; });
    if (result == component_containers_.end()) {
        return false;
    }
    // tear down the component
    RTT::TaskContext* task = (*result).task_context;
    task->stop();
    task->cleanup();
    delete task;
    delete (*result).activity;

    // update the remaining components
    analyzeDependencies();
    connectPorts();
    generateRTOrder();
    setSlavesOnMainContext();

    // if allowed, restart automatically
    if (settings_.mode == Mode::NO_WAIT) {
        activateRTLoop();
    }

    // this always succeeds, if the component the remove was found
    return true;
};

void RTRunner::setSlavesOnMainContext() {
    // NOTE: main context needs to be stopped when calling this

    std::vector<RTT::extras::SlaveActivity*> slaves;
    for (const auto& component : rt_order_) {
        slaves.push_back(component->activity);
    }
    main_context_.clearSlaves();
    main_context_.setSlaves(slaves);
};
    /*
    void RTRunner::stopComponents() {
        main_context_.stop();
        for (auto& component : rt_order) {
            component.task_context->stop();
        }
    };
    */

    void RTRunner::analyzeDependencies() {
    // discard previously constructed stuff
    internal_connections_.clear();
    component_predecessors_.clear();
    component_successors_.clear();

    // sort all components according to name to ensure determinism
    std::sort(component_containers_.begin(), component_containers_.end(),
              [](const auto& a, const auto& b) { return (a.attributes.name > b.attributes.name); });

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
                        ROS_INFO_STREAM("Connected " << c_from.attributes.name << " (" << p_output.original_name
                                                     << ") with " << c_to.attributes.name << " ("
                                                     << p_output.original_name << ") via " << p_input.mapped_name);
                        // 2. predecessor components (aka input dependencies)
                        //    exclude the ignored topics here
                        auto regex = std::regex(c_to.attributes.topics_ignore_for_graph);
                        if (!std::regex_match(p_input.mapped_name, regex)) {
                            if (component_predecessors_[&c_to].insert(&c_from).second) {
                                ROS_INFO_STREAM("Added " << c_from.attributes.name
                                                         << " as predecessor (dependency) for " << c_to.attributes.name
                                                         << ".");
                            }
                        } else {
                            ROS_INFO_STREAM("Ignored " << c_from.attributes.name << " as predecessor (dependency) for "
                                                       << c_to.attributes.name << " due to explicit exclusion.");
                        }
                        // 3. successor components (aka output dependencies)
                        if (component_successors_[&c_from].insert(&c_to).second) {
                            ROS_INFO_STREAM("Added " << c_to.attributes.name << " as successorfor "
                                                     << c_from.attributes.name << ".");
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
        if (component.attributes.is_first || !component_predecessors_[&component].empty()) {
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

    // check if all dependencies are met
    if (L.size() != component_containers_.size()) {
        // error information for debugging
        ROS_ERROR(
            "Cyclic dependencies detected! Check is_first and topics_ignore_for_graph parameters in launchfile. "
            "Following dependencies remained:");
        for (const auto& component : component_containers_) {
            if (!component_predecessors_[&component].empty()) {
                for (const auto& predecessor : component_predecessors_[&component]) {
                    ROS_ERROR_STREAM("Component " << component.attributes.name << " depends on "
                                                  << predecessor->attributes.name);
                }
            }
        }
        return false;
    } else {
        rt_order_ = L;
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
    for (const auto& container : component_containers_) {
        for (const auto& output_port : container.output_ports) {
            output_port.port->disconnect();
        }
        for (const auto& input_port : container.input_ports) {
            input_port.port->disconnect();
        }
    }
}

void RTRunner::connectOrocosPorts() {
    for (const auto& connection : internal_connections_) {
        const auto& to_port   = connection.first;
        const auto& from_port = connection.second;
        to_port->port->connectTo(from_port->port);
    }
}

void RTRunner::connectPortsToRos() {
    auto whitelist = std::regex(settings_.ros_mapping_whitelist);
    auto blacklist = std::regex(settings_.ros_mapping_blacklist);

    auto check_name = [&whitelist, &blacklist](const std::string& name) {
        // elements that match whitelist
        if (std::regex_match(name, whitelist)) {
            // but not blacklist are passed through
            // (empty blacklist means that it never matches)
            if (!std::regex_match(name, blacklist)) {
                return true;
            }
        }
        return false;
    };

    // all output ports that meet whitelist/blacklist are always linked to ROS
    for (const auto& container : component_containers_) {
        for (const auto& output_port : container.output_ports) {
            if (check_name(output_port.mapped_name)) {
                output_port.port->createStream(rtt_roscomm::topic(output_port.mapped_name));
                ROS_INFO_STREAM("Connected " << container.attributes.name << " (output " << output_port.original_name
                                             << ") to ROS topic " << output_port.mapped_name << ".");
            }
        }
    }
    // all input ports that meet whitelist/blacklist
    // AND that do not have an internal connection to an output port are linked to ROS
    // this is required to avoid side-channels through ROS
    for (const auto& container : component_containers_) {
        for (const auto& input_port : container.input_ports) {
            if (check_name(input_port.mapped_name)) {
                // check for internal connection
                if (internal_connections_.count(&input_port) == 0) {
                    input_port.port->createStream(rtt_roscomm::topic(input_port.mapped_name));
                    ROS_INFO_STREAM("Connected " << container.attributes.name << " (input " << input_port.original_name
                                                 << ") to ROS topic " << input_port.mapped_name << ".");
                } else {
                    ROS_INFO_STREAM("Skipped connecting " << container.attributes.name << " (input "
                                                          << input_port.original_name << ") to ROS topic "
                                                          << input_port.mapped_name << " due to internal connection.");
                }
            }
        }
    }
}