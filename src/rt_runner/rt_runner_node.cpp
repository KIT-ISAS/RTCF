#include "rt_runner_node.hpp"
#include <iostream>
#include <memory>
#include "ros/service_server.h"
#include "rt_runner.hpp"

#include <rtt/os/main.h>

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

RTRunnerNode::RTRunnerNode(const ros::NodeHandle &node_handle)
    : node_handle_(node_handle) {
    rt_runner_ = std::make_shared<RTRunner>();
};

RTRunnerNode::~RTRunnerNode() { shutdown(); };

void RTRunnerNode::configure() {
    setupROS();
    loadROSParameters();
    rt_runner_->configure();
};
void RTRunnerNode::shutdown() {
    rt_runner_->shutdown();
    shutdownROS();
};

int RTRunnerNode::loop() {
    ros::spin();
    shutdown();
    ros::spinOnce();

    return 0;
};

void RTRunnerNode::setupROS() {
    loadOrocosComponentService = node_handle_.advertiseService(
        "/rt_runner/load_orocos_component", &RTRunnerNode::loadOrocosComponentCallback,
        this);
    unloadOrocosComponentService = node_handle_.advertiseService(
        "/rt_runner/unload_orocos_component", &RTRunnerNode::unloadOrocosComponentCallback,
        this);
    activateRTLoopService = node_handle_.advertiseService(
        "/rt_runner/activate_rt_loop", &RTRunnerNode::activateRTLoopCallback, this);
    deactivateRTLoopService = node_handle_.advertiseService(
        "/rt_runner/deactivate_rt_loop", &RTRunnerNode::deactivateRTLoopCallback, this);
};

void RTRunnerNode::shutdownROS() {
    loadOrocosComponentService.shutdown();
    unloadOrocosComponentService.shutdown();
    activateRTLoopService.shutdown();
    deactivateRTLoopService.shutdown();
};

void RTRunnerNode::loadROSParameters(){};

bool RTRunnerNode::loadOrocosComponentCallback(
    rtcf::LoadOrocosComponent::Request &req,
    rtcf::LoadOrocosComponent::Response &res) {
    /* TODO: do something here <25-01-21, Stefan Geyer> */

    ROS_DEBUG_STREAM("load service got called");
    ROS_DEBUG_STREAM("got component name: " << req.component_name.data
                                            << std::endl);
    ROS_DEBUG_STREAM("got component rt type: " << req.component_type.data
                                               << std::endl);
    ROS_DEBUG_STREAM("got is rt start point: " << (req.is_start.data == true)
                                               << std::endl);
    for (auto m : req.mappings) {
        ROS_DEBUG_STREAM("got topic mapping: from: ["
                         << m.from_topic.data << "] to: [" << m.to_topic.data
                         << "]" << std::endl);
    }
    res.success.data = true;
    return true;
};

bool RTRunnerNode::unloadOrocosComponentCallback(
    rtcf::UnloadOrocosComponent::Request &req,
    rtcf::UnloadOrocosComponent::Response &res) {
    /* TODO:  <25-01-21, Stefan Geyer> */
    ROS_DEBUG_STREAM("unload service got called");
    ROS_DEBUG_STREAM("got component name: " << req.component_name.data
                                            << std::endl);
    return true;
};

bool RTRunnerNode::activateRTLoopCallback(rtcf::ActivateRTLoop::Request &req,
                                          rtcf::ActivateRTLoop::Response &res) {
    rt_runner_->activateRTLoop();

    return true;
};

bool RTRunnerNode::deactivateRTLoopCallback(
    rtcf::DeactivateRTLoop::Request &req,
    rtcf::DeactivateRTLoop::Response &res) {
    rt_runner_->deactivateRTLoop();

    return true;
};

int ORO_main(int argc, char **argv) {
    ros::init(argc, argv, "RTRunner");
    ros::NodeHandle nh("~");

    RTRunnerNode node = RTRunnerNode(nh);
    node.configure();

    return node.loop();
}
