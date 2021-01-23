#include "rt_runner_node.hpp"
#include "rt_runner.hpp"
#include <iostream>
#include <memory>

#include <rtt/os/main.h>

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rtcf/StartOrocosComponent.h"

RTRunnerNode::RTRunnerNode(const ros::NodeHandle& node_handle)
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
};

void RTRunnerNode::shutdownROS() {
};

void RTRunnerNode::loadROSParameters() {};
    
    void loadOrocosComponentCallback();
    void unloadOrocosComponentCallback();
    void activateRTLoopCallback();
    void deactivateRTLoopCallback();



int ORO_main(int argc, char** argv) {
    ros::init(argc, argv, "RTRunner");
    ros::NodeHandle nh("~");

    RTRunnerNode node = RTRunnerNode(nh);
    node.configure();

    return node.loop();
}

