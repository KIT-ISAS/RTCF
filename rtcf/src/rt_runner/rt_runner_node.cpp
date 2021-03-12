#include "rt_runner_node.hpp"

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <rtt/os/main.h>
#include <signal.h>

#include <iostream>
#include <memory>
#include <vector>

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/spinner.h"
#include "rt_runner.hpp"

RTRunnerNode::RTRunnerNode(const ros::NodeHandle &node_handle, const ros::NodeHandle &node_handle_services,
                           std::shared_ptr<ros::CallbackQueue> queue_services_ptr)
    : node_handle_(node_handle), node_handle_services_(node_handle_services) {
    queue_services_ptr_ = queue_services_ptr;
    // node_handle_services_.setCallbackQueue(&queue_services_);
    rt_runner_ = std::make_shared<RTRunner>();
};

RTRunnerNode::~RTRunnerNode() { shutdown(); };

void RTRunnerNode::configure() {
    loadROSParameters();
    rt_runner_->configure();
    setupROSServices();
};
void RTRunnerNode::shutdown() {
    shutdownROSServices();
    rt_runner_->shutdown();
};

int RTRunnerNode::loop() {
    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        queue_services_ptr_->callOne(ros::WallDuration(0.1));
    }

    shutdown();

    return 0;
};

void RTRunnerNode::setupROSServices() {
    loadOrocosComponentService = node_handle_services_.advertiseService(
        "/rt_runner/load_orocos_component", &RTRunnerNode::loadOrocosComponentCallback, this);
    unloadOrocosComponentService = node_handle_services_.advertiseService(
        "/rt_runner/unload_orocos_component", &RTRunnerNode::unloadOrocosComponentCallback, this);
    activateRTLoopService   = node_handle_services_.advertiseService("/rt_runner/activate_rt_loop",
                                                                   &RTRunnerNode::activateRTLoopCallback, this);
    deactivateRTLoopService = node_handle_services_.advertiseService("/rt_runner/deactivate_rt_loop",
                                                                     &RTRunnerNode::deactivateRTLoopCallback, this);
};

void RTRunnerNode::shutdownROSServices() {
    loadOrocosComponentService.shutdown();
    unloadOrocosComponentService.shutdown();
    activateRTLoopService.shutdown();
    deactivateRTLoopService.shutdown();
};

void RTRunnerNode::loadROSParameters() {
    std::string mode;
    if (node_handle_.getParam("mode", mode)) {
        ROS_INFO_STREAM("RTRunner mode is:  " << mode);
        rt_runner_->setMode(mode);
    }

    int num_components_expected;
    if (node_handle_.getParam("num_components_expected", num_components_expected)) {
        ROS_INFO_STREAM("RTRunner num components expected: " << num_components_expected);
        rt_runner_->setNumComponentsExpected(num_components_expected);
    }

    std::string whitelist_ros_mapping;
    if (node_handle_.getParam("whitelist_ros_mapping", whitelist_ros_mapping)) {
        ROS_INFO_STREAM("RTRunner whitelist for ros mapping: " << whitelist_ros_mapping);
        rt_runner_->setWhitelistRosMapping(whitelist_ros_mapping);
    }

    std::string topics_ignore_for_graph;
    if (node_handle_.getParam("topics_ignore_for_graph", topics_ignore_for_graph)) {
        ROS_INFO_STREAM("RTRunner got list for topics in graph to ignore: " << whitelist_ros_mapping);
        rt_runner_->setTopicsIgnoreForGraph(topics_ignore_for_graph);
    }

    float frequency;
    if (node_handle_.getParam("frequency", frequency)) {
        ROS_INFO_STREAM("RTRunner frequency is: " << frequency);
        rt_runner_->setFrequency(frequency);
    }

    float period;
    if (node_handle_.getParam("period", period)) {
        ROS_INFO_STREAM("RTRunner period is: " << period);
        rt_runner_->setPeriod(period);
    }
};

bool RTRunnerNode::loadOrocosComponentCallback(rtcf::LoadOrocosComponent::Request &req,
                                               rtcf::LoadOrocosComponent::Response &res) {
    /* TODO: do something here <25-01-21, Stefan Geyer> */

    const std::string name                    = req.component_name.data;
    const std::string rt_type                 = req.component_type.data;
    const std::string ns                      = req.ns.data;
    const std::string topics_ignore_for_graph = req.topics_ignore_for_graph.data;
    const bool is_first                       = req.is_first.data;
    const bool is_sync                        = req.is_sync.data;
    std::vector<mapping> mappings;
    for (auto m : req.mappings) {
        mapping mapping;
        mapping.from_topic = m.from_topic.data;
        mapping.to_topic   = m.to_topic.data;
        mappings.push_back(mapping);
    }

    ROS_DEBUG_STREAM("load service got called");
    ROS_DEBUG_STREAM("got component name: " << req.component_name.data << std::endl);
    ROS_DEBUG_STREAM("got component rt type: " << req.component_type.data << std::endl);
    ROS_DEBUG_STREAM("got is_rt_start point: " << (req.is_first.data == true) << std::endl);
    ROS_DEBUG_STREAM("got is_sync: " << (req.is_sync.data == true) << std::endl);
    ROS_DEBUG_STREAM("got namespace name: " << ns << std::endl);
    ROS_DEBUG_STREAM("got topics to ignore for graph: " << topics_ignore_for_graph << std::endl);
    for (auto m : mappings) {
        ROS_DEBUG_STREAM("got topic mapping: from: [" << m.from_topic << "] to: [" << m.to_topic << "]" << std::endl);
    }

    res.success.data =
        rt_runner_->loadOrocosComponent(rt_type, name, ns, topics_ignore_for_graph, is_first, is_sync, mappings);
    return true;
};

bool RTRunnerNode::unloadOrocosComponentCallback(rtcf::UnloadOrocosComponent::Request &req,
                                                 rtcf::UnloadOrocosComponent::Response &res) {
    const std::string name = req.component_name.data;
    const std::string ns   = req.ns.data;

    ROS_DEBUG_STREAM("unload service got called");
    ROS_DEBUG_STREAM("got component name: " << name << std::endl);
    ROS_DEBUG_STREAM("got namespace name: " << ns << std::endl);

    res.success.data = rt_runner_->unloadOrocosComponent(name, ns);
    return true;
};

bool RTRunnerNode::activateRTLoopCallback(rtcf::ActivateRTLoop::Request &req, rtcf::ActivateRTLoop::Response &res) {
    rt_runner_->activateRTLoop();

    return true;
};

bool RTRunnerNode::deactivateRTLoopCallback(rtcf::DeactivateRTLoop::Request &req,
                                            rtcf::DeactivateRTLoop::Response &res) {
    rt_runner_->deactivateRTLoop();

    return true;
};

void RTRunnerNode::stopComponents() { rt_runner_->stopComponents(); };

void sigintHandler(int sig) {
    ROS_DEBUG("sigint handler called");
    node_ptr->stopComponents();
    ros::shutdown();
}

int ORO_main(int argc, char **argv) {
    ros::init(argc, argv, "RTRunner");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_services;

    auto queue_services_ptr = std::make_shared<ros::CallbackQueue>();
    nh_services.setCallbackQueue(queue_services_ptr.get());

    node_ptr = std::make_unique<RTRunnerNode>(nh, nh_services, queue_services_ptr);
    signal(SIGINT, sigintHandler);

    node_ptr->configure();

    return node_ptr->loop();
}
