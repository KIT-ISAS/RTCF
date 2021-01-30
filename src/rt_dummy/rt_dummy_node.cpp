#include "rt_dummy_node.hpp"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/mapping.h"
#include <sstream>
#include <string>
#include <vector>
#include <regex>

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

    /* TODO: move into function <30-01-21, Stefan Geyer> */
    for (auto mapping : mappings) {
        rtcf::mapping m;

        std::stringstream ss_from;
        ss_from << mapping.first;
        m.from_topic.data = ss_from.str();

        std::stringstream ss_to;
        ss_to << mapping.first;
        m.to_topic.data = ss_to.str();

        srv.request.mappings.push_back(m);
    }


    if (loadInRTRunnerClient.call(srv)) {
        ROS_INFO("client called successfully");
    } else {
        //ROS_ERROR("Failed to call service add_two_ints");
        /* TODO: block for service to come online <26-01-21, Stefan Geyer> */
    }
};

void RTDummyNode::unloadInRTRunner() {
    /* TODO:  <25-01-21, Stefan Geyer> */
    rtcf::UnloadOrocosComponent srv;
    if (unloadInRTRunnerClient.call(srv)) {
        ROS_INFO("client called successfully");
    } else {
        //ROS_ERROR("Failed to call service add_two_ints");
        /* TODO: block for service to come online <26-01-21, Stefan Geyer> */
    }
};

void RTDummyNode::handleRemapping(std::vector<std::string> argv) {
    std::regex from_regex("(^[a-zA-Z0-9/].+):=[a-zA-Z0-9/].+$");
    std::regex to_regex("^[a-zA-Z0-9/].+:=([a-zA-Z0-9/].+$)");


    for (const auto s : argv) {
        std::smatch from_regex_match;
        std::smatch to_regex_match;

        bool from_match_found;
        bool to_match_found;

        from_match_found = std::regex_match(s, from_regex_match, from_regex);
        to_match_found = std::regex_match(s, to_regex_match, to_regex);

        if (from_match_found && to_match_found) {
            std::pair<std::string, std::string> mapping;

            mapping.first = from_regex_match[1];
            mapping.second = to_regex_match[1];

            mappings.push_back(mapping);
        }
    }
};

int main(int argc, char **argv) {
    // Set up ROS.

    // necessary because ros::init is absorging argv
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) {
        args.push_back(argv[i]);
    }

    ros::init(argc, argv, "RTDummy");
    ros::NodeHandle nh("~");


    RTDummyNode node = RTDummyNode(nh);
    node.configure();
    node.handleRemapping(args);

    return node.loop();

    return 0;
}  // end main()

