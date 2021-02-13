#include "rt_runner.hpp"
#include <dlfcn.h>
#include <iostream>
#include "rtcf_types.hpp"
#include "rtt/extras/SlaveActivity.hpp"

#include "ros/ros.h"

#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include <vector>
#include <regex>

#include <rtt_roscomm/rostopic.h>
#include <rtt_ros/rtt_ros.h>

RTRunner::RTRunner() : main_context_("main_context"){};

void RTRunner::configure() {
    if (mode_ == "active") {
        isActive = true;
    }
    main_activity_ = new RTT::Activity(ORO_SCHED_RT, 98);
    main_activity_->setPeriod(period_);
    main_context_.setActivity(main_activity_);
    main_context_.configure();
};
void RTRunner::shutdown() { deactivateRTLoop(); };

void RTRunner::activateRTLoop() {
    main_activity_->setPeriod(period_);
    main_context_.start();
    isActive = true;
};

void RTRunner::deactivateRTLoop() {
    main_context_.stop();
    isActive = false;
};

    bool RTRunner::loadOrocosComponent(std::string componentType,
                             std::string componentName, std::string ns,
                             bool is_start, std::vector<mapping> mappings) {
    main_context_.stop();

    rtt_ros::import(componentType);

    RTT::TaskContext* task;
    bool error = createFromLibrary(componentType, componentName, task);

    RTT::extras::SlaveActivity* slave_activity =
        new RTT::extras::SlaveActivity(main_activity_);

    task->setActivity(slave_activity);
    task->configure();
    task->start();

    OrocosContainer orocos_container(componentType, componentName, ns, is_start, mappings, task, slave_activity);

    orocosContainer_.push_back(orocos_container);

    disconnectAllPorts();
    generateRTOrder();
    connectPorts();

    setSlavesOnMainContext();

    if (isActive || ((RTOrder.size()==num_components_expected_) && (mode_=="wait_for_components"))) {
        isActive = true;
        main_context_.start();
    }

    return error;
};

void RTRunner::setSlavesOnMainContext() {
    main_context_.stop();

    /* TODO: this has to be replaced <23-01-21, Stefan Geyer> */

    std::vector<RTT::extras::SlaveActivity*> slaves;

    for (auto orocos_container : RTOrder) {
        slaves.push_back(orocos_container.activity_);
    }

    main_context_.clearSlaves();
    main_context_.setSlaves(slaves);
};

bool RTRunner::createFromLibrary(std::string componentType,
                                 std::string componentName,
                                 RTT::TaskContext*& task) {

    // Try all paths in LD_LIBRARY_PATH to load the orocos component
    const std::string LD_LIBRARY_PATH = getenv("LD_LIBRARY_PATH");
    std::stringstream ss(LD_LIBRARY_PATH);
    std::string item;
    std::vector<std::string> paths;
    while (std::getline(ss, item, ':')) {
        paths.push_back(item);
    }

    ROS_INFO_STREAM("Opening " + componentName + "...\n");

    void* handle;
    for (const auto base_path : paths) {
        std::string path = base_path + "/orocos/gnulinux/" + componentType + "/lib" + componentType + "-gnulinux.so";
        ROS_INFO_STREAM("trying path to load orcos component: " << path);
        handle = dlopen(path.c_str(), RTLD_NOW);
        if (handle) {
            break;
        }
    }

    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return false;
    }

    typedef RTT::TaskContext* create_t(std::string);

    // load the symbol
    ROS_INFO_STREAM("Loading symbol " + componentName +" ...\n");
    // reset errors
    dlerror();

    create_t* creat_task = (create_t*)dlsym(handle, "createComponent");

    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        ROS_ERROR_STREAM("Cannot load symbol 'hello': " << dlsym_error << '\n');
        dlclose(handle);
        return false;
    }

    task = creat_task(componentName.c_str());
    return true;
};

bool RTRunner::unloadOrocosComponent(std::string componentName, std::string ns) {
    /* TODO:  <01-02-21, Stefan Geyer> */
    main_context_.stop();

    disconnectAllPorts();
    generateRTOrder();
    connectPorts();

    setSlavesOnMainContext();

    if (isActive) {
        main_context_.start();
    }

    return true;
};

void RTRunner::generateRTOrder() {
    RTOrder.clear();
    active_graph_.clear();
    active_graph_ = buildGraph();

    if (active_graph_.empty()) {
        return;
    }

    std::vector<GraphOrocosContainer*> queue;

    for (GraphOrocosContainer& n : active_graph_) {
        if (n.is_start_) {
            n.is_queued = true;
            queue.push_back(&n);
        }
    }

    if (queue.empty()) {
        GraphOrocosContainer* start_container = &(active_graph_.at(0));
        start_container->is_queued = true;
        start_container->is_start_ = true;
        queue.push_back(start_container);
    }

    while (active_graph_.size() != RTOrder.size()) {
        ROS_INFO_STREAM("----");
        for (const auto& node : queue) {
            ROS_INFO_STREAM("queue is: " << node->componentName_);
        }

        bool outer_loop_finised = true;
        bool inner_loop_finised = true;
        bool innerst_loop_finished = true;

        auto it_outer = std::begin(queue);
        for (; it_outer != std::end(queue); it_outer++) {
            GraphOrocosContainer* active_node = *it_outer;

            if (active_node->is_satisfied() || active_node->is_start_) {
                ROS_DEBUG_STREAM("active node in inner loop is: "
                                 << (*it_outer)->componentName_);
                std::vector<GraphOrocosContainer*> to_enque =
                    active_node->enqueue_and_satisfy_nodes();
                // TODO: check if erase works as expected <03-02-21, Stefan
                // Geyer>
                RTOrder.push_back(*active_node);
                queue.erase(it_outer);
                for (GraphOrocosContainer* new_queue_element : to_enque) {
                    queue.push_back(new_queue_element);
                }
                outer_loop_finised = false;
                break;
            }
        }

        //// if for loop finished without break
        if (outer_loop_finised) {
            auto it_inner = std::begin(active_graph_);
            for (; it_inner != std::end(active_graph_); it_inner++) {
                GraphOrocosContainer& active_node = *it_inner;
                ROS_DEBUG_STREAM("active node in outer loop 1 is: "
                                 << active_node.componentName_);
                if ((active_node.is_satisfied()) && (!active_node.is_queued)) {
                    std::vector<GraphOrocosContainer*> to_enque =
                        active_node.enqueue_and_satisfy_nodes();
                    // TODO: check if erase works as expected <03-02-21, Stefan
                    // Geyer>
                    active_node.is_queued = true;
                    RTOrder.push_back(active_node);
                    for (GraphOrocosContainer* new_queue_element : to_enque) {
                        queue.push_back(new_queue_element);
                    }
                    inner_loop_finised = false;
                    break;
                }
            }

            if (inner_loop_finised) {
                if (!queue.empty()) {
                    GraphOrocosContainer* active_node = *queue.begin();
                    ROS_DEBUG_STREAM("active node in outer loop 2 is: "
                                     << active_node->componentName_);
                    std::vector<GraphOrocosContainer*> to_enque =
                        active_node->enqueue_and_satisfy_nodes();
                    RTOrder.push_back(*active_node);
                    queue.erase(queue.begin());
                    for (GraphOrocosContainer* new_queue_element : to_enque) {
                        queue.push_back(new_queue_element);
                    }
                    innerst_loop_finished = false;

                } else {
                    auto it_innerst = std::begin(active_graph_);
                    for (; it_innerst != std::end(active_graph_);
                         it_innerst++) {
                        GraphOrocosContainer& active_node = *it_innerst;
                        if (!active_node.is_queued) {
                            std::vector<GraphOrocosContainer*> to_enque =
                                active_node.enqueue_and_satisfy_nodes();
                            active_node.is_queued = true;
                            RTOrder.push_back(active_node);
                            for (GraphOrocosContainer* new_queue_element :
                                 to_enque) {
                                queue.push_back(new_queue_element);
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    for (const auto& node : RTOrder) {
        ROS_INFO_STREAM("RTOrder is: " << node.componentName_);
    }
};

void RTRunner::connectPorts() {
    connectOrocosPorts();
    connectPortsToRos();
}
void RTRunner::connectOrocosPorts() {
    for (GraphOrocosContainer orocos_container : RTOrder) {
        for (GraphOutportContainer outport : orocos_container.output_ports_) {
            if (outport.is_connected) {
                for (GraphPortMatch inport_match : outport.inport_matches) {
                    bool worked = outport.port_->connectTo( inport_match.corr_port_ptr_->port_);
                     ROS_INFO_STREAM(worked <<
                    " connected ports: "
                    << orocos_container.componentName_ << " / "
                    << outport.port_->getName()
                    << " and: " <<
                    inport_match.corr_orocos_ptr_->componentName_
                    << " / " <<
                    inport_match.corr_port_ptr_->port_->getName());
                }
            }
        }
    }
}

void RTRunner::connectPortsToRos() {
    auto whitelist = std::regex(whitelist_ros_mapping_);
    for (GraphOrocosContainer orocos_container : RTOrder) {
        for (GraphOutportContainer outport : orocos_container.output_ports_) {
            if ( std::regex_match(outport.mapping_name_, whitelist)) {
                outport.port_->createStream(rtt_roscomm::topic(outport.mapping_name_));
                ROS_INFO_STREAM("connected orocos outport: " << outport.mapping_name_ << " with ros topic: " << outport.mapping_name_);
            }
        }

        for (GraphInportContainer inport : orocos_container.input_ports_) {
            if ( std::regex_match(inport.mapping_name_, whitelist)) {
                inport.port_->createStream(rtt_roscomm::topic(inport.mapping_name_));
                ROS_INFO_STREAM("connected orocos inport: " << inport.mapping_name_ << " with ros topic: " << inport.mapping_name_);
            }
        }
    }

}


void RTRunner::disconnectAllPorts() {
  for (GraphOrocosContainer orocos_container : RTOrder) {
    for (GraphOutportContainer outport : orocos_container.output_ports_) {
      if (outport.is_connected) {
        outport.port_->disconnect();
      }
    }
  }
}

GraphOrocosContainers RTRunner::buildGraph() {
    GraphOrocosContainers graph;
    for (auto& container : orocosContainer_) {
        graph.push_back(GraphOrocosContainer(container));
    }

    // Mandatory sorting for determenism.
    std::sort(graph.begin(), graph.end(), [](const auto& a, const auto& b) {
        return (a.componentName_ > b.componentName_);
    });

    // Try to connect each outport with each inport.
    // To many for loops. There should be a solution with
    // standard algorithms.
    for (GraphOrocosContainer& start_node : graph) {
        for (GraphOutportContainer& out_port : start_node.output_ports_) {
            for (GraphOrocosContainer& end_node : graph) {
                for (GraphInportContainer& in_port : end_node.input_ports_) {
                    if (out_port.mapping_name_ == in_port.mapping_name_) {
                        ROS_INFO_STREAM("connecte: "
                                        << start_node.componentName_ << " | "
                                        << out_port.original_name_
                                        << " with: " << end_node.componentName_
                                        << " | " << in_port.original_name_);

                        start_node.connected_container_.push_back(&end_node);
                        in_port.is_connected = true;
                        out_port.is_connected = true;

                        in_port.outport_match.corr_orocos_ptr_ = &start_node;
                        in_port.outport_match.corr_port_ptr_ = &out_port;

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
    for (auto& component : RTOrder) {
        component.taskContext_->stop();
    }
};

void RTRunner::setMode(std::string mode) { mode_ = mode; }
void RTRunner::setNumComponentsExpected(int num) {
    num_components_expected_ = num;
};
void RTRunner::setWhitelistRosMapping(std::string whitelist) {
    whitelist_ros_mapping_ = whitelist;
};
void RTRunner::setFrequency(float frequency) { period_ = 1.0 / frequency; };
void RTRunner::setPeriod(float period) { period_ = period; };
