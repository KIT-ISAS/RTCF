#include "rt_runner.hpp"
#include <dlfcn.h>
#include <iostream>
#include "rtcf_types.hpp"
#include "rtt/extras/SlaveActivity.hpp"

#include "ros/ros.h"

#include <ocl/OCL.hpp>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include <vector>

RTRunner::RTRunner() : main_context_("main_context"){};

void RTRunner::configure() {
    isActive = false;
    period_ = 1.0;
    main_activity_ = new RTT::Activity(ORO_SCHED_RT, 98);
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
                                   std::string componentName, bool is_start,
                                   std::vector<mapping> mappings) {
    main_context_.stop();

    RTT::TaskContext* task;
    bool error = createFromLibrary(componentType, componentName, task);

    RTT::extras::SlaveActivity* slave_activity =
        new RTT::extras::SlaveActivity(main_activity_);

    task->setActivity(slave_activity);
    task->configure();
    task->start();

    OrocosContainer orocos_container(componentType, componentName, is_start, mappings, task, slave_activity);

    orocosContainer_.push_back(orocos_container);

    disconnectPorts();
    generateRTOrder();
    connectPorts();

    setSlavesOnMainContext();

    if (isActive) {
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
    if (isActive) {
        main_context_.start();
    }
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

bool RTRunner::unloadOrocosComponent(std::string componentName) {
    /* TODO:  <01-02-21, Stefan Geyer> */
    main_context_.stop();

    disconnectPorts();
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
    GraphOrocosContainers graph = buildGraph();

    if (graph.empty()) {
        return;
    }

    std::vector<GraphOrocosContainer*> queue;

    for (GraphOrocosContainer& n : graph) {
        if (n.is_start_) {
            n.is_queued = true;
            queue.push_back(&n);
        }
    }

    if (queue.empty()) {
        GraphOrocosContainer* start_container = &(graph.at(0));
        start_container->is_queued = true;
        start_container->is_start_ = true;
        queue.push_back(start_container);
    }

    while (graph.size() != RTOrder.size()) {
        // for (const auto& node : queue) {
        //[> TODO: hier morgen weiter machen <03-02-21, Stefan Geyer> <]
        // ROS_INFO_STREAM("queue is: " << node.componentName_);
        //}
        //
        bool outer_loop_finised = true;
        bool inner_loop_finised = true;
        bool innerst_loop_finished = true;

        auto it_outer = std::begin(queue);
        for (; it_outer != std::end(queue); it_outer++) {
            GraphOrocosContainer* active_node = *it_outer;

            if (active_node->is_satisfied() || active_node->is_start_) {
                ROS_INFO_STREAM(
                    "active node is: " << (*it_outer)->componentName_);
                std::cout << "debug 1" << std::endl;
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
            auto it_inner = std::begin(graph);
            for (; it_inner != std::end(graph); it_inner++) {
                std::cout << "debug 2" << std::endl;
                GraphOrocosContainer& active_node = *it_inner;
                if ((active_node.is_satisfied()) && (!active_node.is_queued)) {
                    std::cout << "debug 2.1" << std::endl;
                    std::vector<GraphOrocosContainer*> to_enque =
                        active_node.enqueue_and_satisfy_nodes();
                    // TODO: check if erase works as expected <03-02-21, Stefan
                    // Geyer>
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
                    std::vector<GraphOrocosContainer*> to_enque =
                        active_node->enqueue_and_satisfy_nodes();
                    ////TODO: check if erase works as expected <03-02-21, Stefan
                    /// Geyer>
                    RTOrder.push_back(*active_node);
                    queue.erase(queue.begin());
                    for (GraphOrocosContainer* new_queue_element : to_enque) {
                        queue.push_back(new_queue_element);
                    }
                    innerst_loop_finished = false;

                } else {
                    auto it_innerst = std::begin(graph);
                    for (; it_innerst != std::end(graph); it_innerst++) {
                        GraphOrocosContainer& active_node = *it_innerst;
                        if (!active_node.is_queued) {
                            std::vector<GraphOrocosContainer*> to_enque =
                                active_node.enqueue_and_satisfy_nodes();
                            // TODO: check if erase works as expected <03-02-21,
                            // Stefan Geyer>
                            active_node.is_queued=true;
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
    for (GraphOrocosContainer orocos_container : RTOrder) {
        for (GraphOutportContainer outport : orocos_container.output_ports_) {
            if (outport.is_connected) {
                for (GraphPortMatch inport_match : outport.inport_matches) {
                    outport.port_->connectTo(
                        inport_match.corr_port_ptr_->port_);
                     ROS_INFO_STREAM(
                    "connected ports: "
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

void RTRunner::disconnectPorts() {
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
    for (const auto& container : orocosContainer_) {
        graph.push_back(GraphOrocosContainer(container));
    }

    // Mandatory sorting for determenism.
    std::sort(graph.begin(), graph.end(), [](const auto& a, const auto& b) {
        return (a.componentName_ > b.componentName_);
    });

    // Try to connect each outport with each inport.
    // To many for loops. There should be a solution with
    // standard algorithms.
    for (auto& start_node : graph) {
        for (auto& out_port : start_node.output_ports_) {
            for (auto& end_node : graph) {
                for (auto& in_port : end_node.input_ports_) {
                    if (out_port.mapping_name_ == in_port.mapping_name_) {
                        ROS_INFO_STREAM("connecte: "
                                        << start_node.componentName_ << " | "
                                        << out_port.original_name_
                                        << " with: " << end_node.componentName_
                                        << " | " << in_port.original_name_);

                        start_node.connected_container_.push_back(&end_node);
                        in_port.is_connected = true;

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

void RTRunner::setFrequency(float frequency) { period_ = 1 / frequency; };
void RTRunner::setPeriod(float period) { period_ = period; };
