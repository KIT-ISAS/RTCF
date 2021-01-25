#include "rt_dummy_node.hpp"
#include "rtcf/LoadOrocosComponent.h"

RTDummyNode::RTDummyNode(const ros::NodeHandle &node_handle){

};

int RTDummyNode::loop() {
    loadInRTRunner();
    ros::spin();
    unloadInRTRunner();
    shutdown();
    ros::spinOnce();

    return 0;
};

void RTDummyNode::configure(){

};

void RTDummyNode::shutdown() { shutdownROS(); };

int loop();

void RTDummyNode::setupROS() {
    loadInRTRunnerClient =
        node_handle_.serviceClient<rtcf::LoadOrocosComponent>(
            "load_orocos_component");

    unloadInRTRunnerClient =
        node_handle_.serviceClient<rtcf::UnloadOrocosComponent>(
            "unload_orocos_component");
};

void RTDummyNode::shutdownROS() {
    loadInRTRunnerClient.shutdown();
    unloadInRTRunnerClient.shutdown();
};

void loadROSParameters();

void RTDummyNode::loadInRTRunner() {
    /* TODO:  <25-01-21, Stefan Geyer> */
    rtcf::LoadOrocosComponent srv;
    if (loadInRTRunnerClient.call(srv)) {
        ROS_INFO("client called successfully");
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
    }
};

void RTDummyNode::unloadInRTRunner() {
    /* TODO:  <25-01-21, Stefan Geyer> */
    rtcf::UnloadOrocosComponent srv;
    if (unloadInRTRunnerClient.call(srv)) {
        ROS_INFO("client called successfully");
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
    }
};

void handleRemapping(int argc, char **argv);

int main(int argc, char **argv) {
    // Set up ROS.
    ros::init(argc, argv, "RTDummy");
    ros::NodeHandle nh("~");

    RTDummyNode node = RTDummyNode(nh);
    node.configure();

    return node.loop();

    return 0;
}  // end main()

