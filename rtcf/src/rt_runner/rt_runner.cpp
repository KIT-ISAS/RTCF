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
    main_activity_->setPeriod(period_);
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
    if (settings_.mode == Mode::WAIT_FOR_COMPONENTS && rt_order.size() >= settings_.expected_num_components) {
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
    component_containers.push_back(component_container);

    // do the magic (connecting nodes, graph resolution, etc.)
    disconnectPorts();
    generateRTOrder();
    connectPorts();
    setSlavesOnMainContext();

    if ((settings_.mode == Mode::NO_WAIT) ||
        (settings_.mode == Mode::WAIT_FOR_COMPONENTS && rt_order.size() == settings_.expected_num_components)) {
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
    auto result = std::find_if(component_containers.begin(), component_containers.end(),
                               [&info](const auto& container) { return container.attributes.name == info.name; });
    if (result == component_containers.end()) {
        return false;
    }
    // tear down the component
    RTT::TaskContext* task = (*result).task_context;
    task->stop();
    task->cleanup();
    delete task;
    delete (*result).activity;

    // update the remaining components
    generateRTOrder();
    connectPorts();
    setSlavesOnMainContext();

    // if allowed, restart automatically
    if (settings_.mode == Mode::NO_WAIT) {
        activateRTLoop();
    }

    // this always succeeds, if the component the remove was found
    return true;
};

/*
void RTRunner::setSlavesOnMainContext() {
    main_context_.stop();

    // TODO: this has to be replaced <23-01-21, Stefan Geyer>

std::vector<RTT::extras::SlaveActivity*> slaves;

for (auto orocos_container : rt_order) {
    slaves.push_back(orocos_container.activity);
}

main_context_.clearSlaves();
main_context_.setSlaves(slaves);
}
;


void RTRunner::generateRTOrder() {
    rt_order.clear();
    active_graph_.clear();
    active_graph_ = analyzeDependencies();

    if (active_graph_.empty()) {
        return;
    }

    std::vector<GraphOrocosContainer*> queue;

    for (GraphOrocosContainer& n : active_graph_) {
        if (n.is_first_) {
            n.is_queued = true;
            queue.push_back(&n);
        }
    }

    if (queue.empty()) {
        GraphOrocosContainer* start_container = &(active_graph_.at(0));
        start_container->is_queued            = true;
        start_container->is_first_            = true;
        queue.push_back(start_container);
    }

    while (active_graph_.size() != rt_order.size()) {
        ROS_INFO_STREAM("----");
        for (const auto& node : queue) {
            ROS_INFO_STREAM("queue is: " << node->componentName_);
        }

        bool outer_loop_finised   = true;
        bool inner_loop_finised   = true;
        bool innerst_loop_finised = true;

        // Handle nodes which are in queue and are satisfied or start_node
        auto it_outer = std::begin(queue);
        for (; it_outer != std::end(queue); it_outer++) {
            GraphOrocosContainer* active_node = *it_outer;

            if (active_node->is_satisfied() || active_node->is_first_) {
                ROS_DEBUG_STREAM("active node in inner loop is: " << (*it_outer)->componentName_);
                std::vector<GraphOrocosContainer*> to_enque = active_node->enqueue_and_satisfy_nodes();
                // TODO: check if erase works as expected <03-02-21, Stefan
                // Geyer>
                rt_order.push_back(*active_node);
                queue.erase(it_outer);
                for (GraphOrocosContainer* new_queue_element : to_enque) {
                    queue.push_back(new_queue_element);
                }
                outer_loop_finised = false;
                break;
            }
        }

        // if for loop finished without break
        // Handle nodes which are in graph and are satisfied
        if (outer_loop_finised) {
            auto it_inner = std::begin(active_graph_);
            for (; it_inner != std::end(active_graph_); it_inner++) {
                GraphOrocosContainer& active_node = *it_inner;
                ROS_DEBUG_STREAM("active node in outer loop 1 is: " << active_node.componentName_);
                if ((active_node.is_satisfied()) && (!active_node.is_queued)) {
                    std::vector<GraphOrocosContainer*> to_enque = active_node.enqueue_and_satisfy_nodes();
                    // TODO: check if erase works as expected <03-02-21, Stefan
                    // Geyer>
                    active_node.is_queued = true;
                    rt_order.push_back(active_node);
                    for (GraphOrocosContainer* new_queue_element : to_enque) {
                        queue.push_back(new_queue_element);
                    }
                    inner_loop_finised = false;
                    break;
                }
            }

            // Handle first node which is in queue and is not sync
            if (inner_loop_finised) {
                bool queue_is_empty = queue.empty();
                if (!queue_is_empty) {
                    auto it_innerst = std::begin(queue);
                    for (; it_innerst != std::end(queue); it_innerst++) {
                        GraphOrocosContainer* active_node = *it_innerst;

                        if (!active_node->is_sync_) {
                            std::vector<GraphOrocosContainer*> to_enque = active_node->enqueue_and_satisfy_nodes();
                            rt_order.push_back(*active_node);
                            queue.erase(it_innerst);
                            for (GraphOrocosContainer* new_queue_element : to_enque) {
                                queue.push_back(new_queue_element);
                            }
                            innerst_loop_finised = false;
                            break;
                        }
                    }
                }
                // If no other optional is possible Handle first node which
                // is in not in queue yet and handle no matter what
                if (queue_is_empty || innerst_loop_finised == true) {
                    auto it_innerst = std::begin(active_graph_);
                    for (; it_innerst != std::end(active_graph_); it_innerst++) {
                        GraphOrocosContainer& active_node = *it_innerst;
                        if (!active_node.is_queued) {
                            std::vector<GraphOrocosContainer*> to_enque = active_node.enqueue_and_satisfy_nodes();
                            active_node.is_queued                       = true;
                            rt_order.push_back(active_node);
                            for (GraphOrocosContainer* new_queue_element : to_enque) {
                                queue.push_back(new_queue_element);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    for (const auto& node : rt_order) {
        ROS_INFO_STREAM("rt_order is: " << node.componentName_);
    }
};
*/

void RTRunner::connectPorts() {
    connectOrocosPorts();
    connectPortsToRos();
}

void RTRunner::disconnectPorts() {
    // simply disconnect all input and output ports
    // this also stops ROS connections created by createStream()
    for (const auto& container : component_containers) {
        for (const auto& output_port : container.output_ports) {
            output_port.port->disconnect();
        }
        for (const auto& input_port : container.input_ports) {
            input_port.port->disconnect();
        }
    }
}

void RTRunner::connectOrocosPorts() {
    for (const auto& connection : internal_connections) {
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
    for (const auto& container : component_containers) {
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
    for (const auto& container : component_containers) {
        for (const auto& input_port : container.input_ports) {
            if (check_name(input_port.mapped_name)) {
                // check for internal connection
                if (internal_connections.count(&input_port) == 0) {
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

void RTRunner::analyzeDependencies() {
    // sort all components according to name to ensure determinism
    std::sort(component_containers.begin(), component_containers.end(),
              [](const auto& a, const auto& b) { return (a.attributes.name > b.attributes.name); });

    // iterate over all output ports
    for (const auto& c_from : component_containers) {
        for (const auto& p_output : c_from.output_ports) {
            // iterate over all input ports
            for (const auto& c_to : component_containers) {
                for (const auto& p_input : c_to.input_ports) {
                    // check if the ports are connected
                    if (p_output.mapped_name == p_input.mapped_name) {
                        // check whether the current input is supplied by a single source
                        if (internal_connections.find(&p_input) != internal_connections.end()) {
                            ROS_WARN_STREAM(
                                "Found multiple connections to input port "
                                << p_input.mapped_name << " of component " << c_to.attributes.name << ". "
                                << "Therefore, the this connection will be ignored. Check your launchfile!");
                            continue;
                        }
                        // extract relevant information
                        // 1. input ports that are supplied by an output port (aka internal connections)
                        internal_connections[&p_input] = &p_output;
                        // 2. predecessor components (aka input dependencies)
                        component_dependencies[&c_to].insert(&c_from);

                        ROS_INFO_STREAM("Connected " << c_from.attributes.name << " (" << p_output.original_name
                                                     << ") with " << c_to.attributes.name << " ("
                                                     << p_output.original_name << ") via " << p_input.mapped_name);
                    }
                }
            }
        }
    }
}
/*
void RTRunner::stopComponents() {
    main_context_.stop();
    for (auto& component : rt_order) {
        component.task_context->stop();
    }
};
*/