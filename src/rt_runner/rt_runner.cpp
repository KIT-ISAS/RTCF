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
    RTT::TaskContext* task;
    bool error = createFromLibrary(componentType, componentName, task);

    RTT::extras::SlaveActivity* slave_activity =
        new RTT::extras::SlaveActivity(main_activity_);

    task->setActivity(slave_activity);
    task->configure();

    OrocosContainer orocos_container(componentType, componentName, is_start, mappings, task, slave_activity);

    orocosContainer_.push_back(orocos_container);

    generateRTOrder();
    setSlavesOnMainContext();

    return error;
};

void RTRunner::setSlavesOnMainContext() {
    main_context_.stop();

    /* TODO: this has to be replaced <23-01-21, Stefan Geyer> */

    std::vector<RTT::extras::SlaveActivity*> slaves;

    for (auto orocos_container : orocosContainer_) {
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

    generateRTOrder();
    setSlavesOnMainContext();

    if (isActive) {
        main_context_.start();
    }

    return true;
};

void RTRunner::generateRTOrder(){
    GraphOrocosContainers graph = buildGraph();

    if (graph.empty()) {
        return;
    }

    GraphOrocosContainers queue;

    for (GraphOrocosContainer& n : graph) {
        if (n.is_start_) {
            n.is_queued = true;
            queue.push_back(n);
        }
    }

    if (queue.empty()) {
        queue.push_back(graph.at(0));
    }

    while (queue.size() > 0) {
        auto it_outer = std::begin(queue);
        for (; it_outer != std::end(queue); it_outer++) {
            GraphOrocosContainer& active_node = *it_outer;

            if (active_node.is_satisfied() || active_node.is_start_) {
                GraphOrocosContainers to_enque =
                    active_node.enqueue_and_satisfy_nodes();
                /* TODO: check if erase works as expected <03-02-21, Stefan
                 * Geyer> */
                queue.erase(it_outer);
                queue.insert(queue.end(), to_enque.begin(), to_enque.end());
                RTOrder.push_back(active_node);
                break;
            }
        }

        // if for loop finished without break
        if (it_outer == std::end(queue)) {
            auto it_inner = std::begin(graph);
            for (; it_inner != std::end(graph); it_inner++) {
                GraphOrocosContainer active_node = *it_inner;
                if ((active_node.is_satisfied()) && (!active_node.is_queued)){
                GraphOrocosContainers to_enque =
                    active_node.enqueue_and_satisfy_nodes();
                /* TODO: check if erase works as expected <03-02-21, Stefan
                 * Geyer> */
                queue.insert(queue.end(), to_enque.begin(), to_enque.end());
                break;
                }
            }

            if (it_inner == std::end(graph)){
                GraphOrocosContainer &active_node = queue.at(0);
                GraphOrocosContainers to_enque =
                    active_node.enqueue_and_satisfy_nodes();
                /* TODO: check if erase works as expected <03-02-21, Stefan
                 * Geyer> */
                queue.erase(queue.begin());
                queue.insert(queue.end(), to_enque.begin(), to_enque.end());
                RTOrder.push_back(active_node);
            }
        }
    }
};

GraphOrocosContainers RTRunner::buildGraph() {
    GraphOrocosContainers graph;
    for (const auto& container : orocosContainer_) {
        graph.push_back(GraphOrocosContainer(container));
    }

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

    // Mandatory sorting for determenism.
    std::sort(graph.begin(), graph.end(), [](const auto& a, const auto& b) {
        return (a.componentName_ > b.componentName_);
    });

    return graph;
}

void RTRunner::setFrequency(float frequency) { period_ = 1 / frequency; };
void RTRunner::setPeriod(float period) { period_ = period; };
