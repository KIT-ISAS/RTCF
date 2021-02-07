#include "rt_runner_node.hpp"
#include <iostream>
#include <memory>
#include "ros/service_server.h"
#include "rt_runner.hpp"

#include <rtt/os/main.h>
#include <vector>

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

RTRunnerNode::RTRunnerNode(const ros::NodeHandle &node_handle)
    : node_handle_(node_handle) {
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
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    shutdown();

    return 0;
};

void RTRunnerNode::setupROSServices() {
  loadOrocosComponentService = node_handle_.advertiseService(
      "/rt_runner/load_orocos_component",
      &RTRunnerNode::loadOrocosComponentCallback, this);
  unloadOrocosComponentService = node_handle_.advertiseService(
      "/rt_runner/unload_orocos_component",
      &RTRunnerNode::unloadOrocosComponentCallback, this);
  activateRTLoopService = node_handle_.advertiseService(
      "/rt_runner/activate_rt_loop", &RTRunnerNode::activateRTLoopCallback,
      this);
  deactivateRTLoopService = node_handle_.advertiseService(
      "/rt_runner/deactivate_rt_loop", &RTRunnerNode::deactivateRTLoopCallback,
      this);
};

void RTRunnerNode::shutdownROSServices() {
  loadOrocosComponentService.shutdown();
  unloadOrocosComponentService.shutdown();
  activateRTLoopService.shutdown();
  deactivateRTLoopService.shutdown();
};

void RTRunnerNode::loadROSParameters(){
    std::string mode;
    if (node_handle_.getParam("mode", mode)) {
        ROS_INFO_STREAM("RTRunner mode is:  " << mode);
        rt_runner_->setMode(mode);
    }

    int num_components_expected;
    if (node_handle_.getParam("num_components_expected",
                              num_components_expected)) {
        ROS_INFO_STREAM(
            "RTRunner num components expected: " << num_components_expected);
        rt_runner_->setNumComponentsExpected(num_components_expected);
    }

    std::string whitelist_ros_mapping;
    if (node_handle_.getParam("whitelist_ros_mapping", whitelist_ros_mapping)) {
        ROS_INFO_STREAM(
            "RTRunner whitelist for ros mapping: " << whitelist_ros_mapping);
        rt_runner_->setWhitelistRosMapping(whitelist_ros_mapping);
    }

    float frequency;
    if (node_handle_.getParam("frequency", frequency)) {
        ROS_INFO_STREAM(
            "RTRunner frequency is: " << frequency);
        rt_runner_->setFrequency(frequency);
    }

    float period;
    if (node_handle_.getParam("period", period)) {
        ROS_INFO_STREAM(
            "RTRunner period is: " << period);
        rt_runner_->setPeriod(period);

    }
};

bool RTRunnerNode::loadOrocosComponentCallback(
    rtcf::LoadOrocosComponent::Request &req,
    rtcf::LoadOrocosComponent::Response &res) {
    /* TODO: do something here <25-01-21, Stefan Geyer> */

    const std::string name = req.component_name.data;
    const std::string rt_type = req.component_type.data;
    const bool is_start = req.is_start.data;
    std::vector<mapping> mappings;
    for (auto m : req.mappings) {
        mapping mapping;
        mapping.from_topic = m.from_topic.data;
        mapping.to_topic = m.to_topic.data;
        mappings.push_back(mapping);
    }

    ROS_DEBUG_STREAM("load service got called");
    ROS_DEBUG_STREAM("got component name: " << req.component_name.data
                                            << std::endl);
    ROS_DEBUG_STREAM("got component rt type: " << req.component_type.data
                                               << std::endl);
    ROS_DEBUG_STREAM("got is rt start point: " << (req.is_start.data == true)
                                               << std::endl);
    for (auto m : mappings) {
        ROS_DEBUG_STREAM("got topic mapping: from: ["
                         << m.from_topic << "] to: [" << m.to_topic
                         << "]" << std::endl);
    }

    return rt_runner_->loadOrocosComponent(rt_type, name, is_start, mappings);
};

bool RTRunnerNode::unloadOrocosComponentCallback(
    rtcf::UnloadOrocosComponent::Request &req,
    rtcf::UnloadOrocosComponent::Response &res) {

    const std::string name = req.component_name.data;
    ROS_DEBUG_STREAM("unload service got called");
    ROS_DEBUG_STREAM("got component name: " << name << std::endl);

    return rt_runner_->unloadOrocosComponent(name);
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
