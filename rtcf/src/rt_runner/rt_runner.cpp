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

#include "rtcf_types.hpp"

RTRunner::RTRunner() : main_context_("main_context"){};

void RTRunner::configure(const Settings& settings) : settings_(settings), is_active_(false) {
    // TODO: add CPU affinity and similar stuff here
    main_activity_ = new RTT::Activity(ORO_SCHED_RT, 98);
    main_activity_->setPeriod(period_);
    main_context_.setActivity(main_activity_);
    main_context_.configure();
};
void RTRunner::shutdown() { deactivateRTLoop(); };

void RTRunner::activateRTLoop() {
    if (!is_active_) {
        main_activity_->set(1.0 / settings_.frequency);
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
    if (settings_ == Mode::WAIT_FOR_COMPONENTS && rt_order.size() >= settings_.expected_num_components) {
        ROS_ERROR("More components were loaded than expected. Additional components will not be active.");
        return false;
    }

    // load package
    if (!rtt_ros::import(info.rt_package)) {
        ROS_ERROR("Could not load ROS package " << info.rt_package << " into OROCOS.");
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
        ROS_ERROR("configuration() call to component " << component_container.attributes.name << " failed")
        return false;
    }
    // component_container.task_context->start() should not be called here as this would trigger the updateHook()

    // stop the execution now (if not already stopped)
    // it is not necessary to stop the loop earlier because the component does not get triggered
    deactivateRTLoop();
    component_containers.push_back(component_container);

    // do the magic (connecting nodes, graph resolution, etc.)
    disconnectAllPorts();
    generateRTOrder();
    connectPorts();
    setSlavesOnMainContext();

    if ((settings_.mode == Mode::NO_WAIT) ||
        (settings_.mode == Mode::WAIT_FOR_COMPONENTS && rt_order.size() == settings_.expected_num_components)) {
        activateRTLoop();
    }

    return true;
};

TaskContext* createInstance(const std::string component_type, const std::string& component_name) {
    TaskContext* tc = nullptr;
    tc              = RTT::ComponentLoader::Instance()->loadComponent(component_name, component_type);
    return tc;
}

void RTRunner::setSlavesOnMainContext() {
    main_context_.stop();

    /* TODO: this has to be replaced <23-01-21, Stefan Geyer> */

    std::vector<RTT::extras::SlaveActivity*> slaves;

    for (auto orocos_container : rt_order) {
        slaves.push_back(orocos_container.activity_);
    }

    main_context_.clearSlaves();
    main_context_.setSlaves(slaves);
};

bool RTRunner::unloadOrocosComponent(std::string componentName, std::string ns) {
    /* TODO:  <01-02-21, Stefan Geyer> */
    main_context_.stop();

    disconnectAllPorts();

    auto it = std::begin(component_containers);
    for (; it != std::end(component_containers); it++) {
        if (it->componentName_ == componentName) {
            component_containers.erase(it);
        }
    }

    generateRTOrder();
    connectPorts();

    setSlavesOnMainContext();

    if (is_active_) {
        main_context_.start();
    }

    return true;
};

void RTRunner::generateRTOrder() {
    rt_order.clear();
    active_graph_.clear();
    active_graph_ = buildGraph();

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

void RTRunner::connectPorts() {
    connectOrocosPorts();
    connectPortsToRos();
}
void RTRunner::connectOrocosPorts() {
    for (GraphOrocosContainer orocos_container : rt_order) {
        for (GraphOutportContainer outport : orocos_container.output_ports_) {
            if (outport.is_connected) {
                for (GraphPortMatch inport_match : outport.inport_matches) {
                    bool worked = outport.port_->connectTo(inport_match.corr_port_ptr_->port_);
                    ROS_INFO_STREAM(worked << " connected ports: " << orocos_container.componentName_ << " / "
                                           << outport.port_->getName()
                                           << " and: " << inport_match.corr_orocos_ptr_->componentName_ << " / "
                                           << inport_match.corr_port_ptr_->port_->getName());
                }
            }
        }
    }
}

void RTRunner::connectPortsToRos() {
    auto whitelist = std::regex(whitelist_ros_mapping_);
    for (GraphOrocosContainer orocos_container : rt_order) {
        for (GraphOutportContainer outport : orocos_container.output_ports_) {
            if (std::regex_match(outport.mapping_name_, whitelist)) {
                outport.port_->createStream(rtt_roscomm::topic(outport.mapping_name_));
                ROS_INFO_STREAM("connected orocos outport: " << outport.mapping_name_
                                                             << " with ros topic: " << outport.mapping_name_);
            }
        }

        for (GraphInportContainer inport : orocos_container.input_ports_) {
            if (std::regex_match(inport.mapping_name_, whitelist)) {
                if (!inport.is_connected) {
                    inport.port_->createStream(rtt_roscomm::topic(inport.mapping_name_));
                    ROS_INFO_STREAM("connected orocos inport: " << inport.mapping_name_
                                                                << " with ros topic: " << inport.mapping_name_);
                } else {
                    ROS_WARN_STREAM("did not connected orocos inport: "
                                    << inport.mapping_name_ << " with ros topic: " << inport.mapping_name_
                                    << "because a output port is already connected "
                                       "to ros with the same topic. This would cause a "
                                       "unexpected connection between orocos ports through "
                                       "ros");
                }
            }
        }
    }
}

void RTRunner::disconnectAllPorts() {
    for (GraphOrocosContainer orocos_container : rt_order) {
        for (GraphOutportContainer outport : orocos_container.output_ports_) {
            if (outport.is_connected) {
                outport.port_->disconnect();
            }
        }
    }
}

GraphOrocosContainers RTRunner::buildGraph() {
    GraphOrocosContainers graph;
    for (auto& container : component_containers) {
        graph.push_back(GraphOrocosContainer(container));
    }

    // Mandatory sorting for determenism.
    std::sort(graph.begin(), graph.end(),
              [](const auto& a, const auto& b) { return (a.componentName_ > b.componentName_); });

    // Try to connect each outport with each inport.
    // To many for loops. There should be a solution with
    // standard algorithms.
    for (GraphOrocosContainer& start_node : graph) {
        for (GraphOutportContainer& out_port : start_node.output_ports_) {
            for (GraphOrocosContainer& end_node : graph) {
                for (GraphInportContainer& in_port : end_node.input_ports_) {
                    if (out_port.mapping_name_ == in_port.mapping_name_) {
                        ROS_INFO_STREAM("connecte: " << start_node.componentName_ << " | " << out_port.original_name_
                                                     << " with: " << end_node.componentName_ << " | "
                                                     << in_port.original_name_);

                        start_node.connected_container_.push_back(&end_node);
                        in_port.is_connected  = true;
                        out_port.is_connected = true;

                        in_port.outport_match.corr_orocos_ptr_ = &start_node;
                        in_port.outport_match.corr_port_ptr_   = &out_port;

                        GraphPortMatch port_match(&end_node, &in_port);
                        out_port.inport_matches.push_back(port_match);
                    }
                }
            }
        }
    }

    return graph;
}

void RTRunner::stopComponents() {
    main_context_.stop();
    for (auto& component : rt_order) {
        component.taskContext_->stop();
    }
};
