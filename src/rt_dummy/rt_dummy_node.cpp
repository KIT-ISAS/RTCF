#include "rt_dummy_node.hpp"

#include <signal.h>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "ros/duration.h"
#include "ros/service.h"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/mapping.h"

RTDummyNode::RTDummyNode(const ros::NodeHandle &node_handle){

};

int RTDummyNode::loop() {
    loadInRTRunner();
    ros::spin();
    return 0;
};

void RTDummyNode::configure() { setupROS(); };

void RTDummyNode::shutdown() { shutdownROS(); };

void RTDummyNode::setupROS() {
    ros::service::waitForService("/rt_runner/load_orocos_component",
                                 ros::Duration(-1));
    loadInRTRunnerClient =
        node_handle_.serviceClient<rtcf::LoadOrocosComponent>(
            "/rt_runner/load_orocos_component");

    ros::service::waitForService("/rt_runner/unload_orocos_component",
                                 ros::Duration(-1));
    unloadInRTRunnerClient =
        node_handle_.serviceClient<rtcf::UnloadOrocosComponent>(
            "/rt_runner/unload_orocos_component");
};

void RTDummyNode::shutdownROS() {
    loadInRTRunnerClient.shutdown();
    unloadInRTRunnerClient.shutdown();
};

rtcf::LoadOrocosComponent RTDummyNode::genLoadMsg() {
    rtcf::LoadOrocosComponent srv;

    for (auto mapping : dummy_attributes_.mappings) {
        rtcf::mapping m;

        std::stringstream ss_from;
        ss_from << mapping.from_topic;
        m.from_topic.data = ss_from.str();

        std::stringstream ss_to;
        ss_to << mapping.to_topic;
        m.to_topic.data = ss_to.str();

        srv.request.mappings.push_back(m);
    }

    srv.request.component_name.data = dummy_attributes_.name;
    srv.request.component_type.data = dummy_attributes_.rt_type;
    srv.request.is_start.data = dummy_attributes_.is_start;

    return srv;
};

rtcf::UnloadOrocosComponent RTDummyNode::genUnloadMsg() {
    rtcf::UnloadOrocosComponent srv;

    srv.request.component_name.data = dummy_attributes_.name;

    return srv;
};

void loadROSParameters();

void RTDummyNode::loadInRTRunner() {
    rtcf::LoadOrocosComponent srv = genLoadMsg();

    if (loadInRTRunnerClient.call(srv)) {
        ROS_INFO("client called successfully");
    } else {
        ROS_ERROR("Failed to call service load in RT Runner");
    }
};

void RTDummyNode::unloadInRTRunner() {
    rtcf::UnloadOrocosComponent srv = genUnloadMsg();

    if (unloadInRTRunnerClient.call(srv)) {
        ROS_INFO("client called successfully");
    } else {
        ROS_INFO("Failed to call service unload in RT Runner");
    }
};

void RTDummyNode::handleArgs(std::vector<std::string> argv) {
    /*****************************
     *  Handle Topic Remappings  *
     *****************************/

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
            mapping mapping;

            mapping.from_topic = from_regex_match[1];
            mapping.to_topic = to_regex_match[1];

            dummy_attributes_.mappings.push_back(mapping);

            ROS_INFO_STREAM("got remapping from: [" << mapping.from_topic
                                                    << "] to: ["
                                                    << mapping.to_topic << "]");
        }
    }

    /**********************
     *  Handle Node Name  *
     **********************/

    std::regex name_regex("^__name:=(.+$)");

    for (const auto s : argv) {
        std::smatch regex_match;
        if (std::regex_match(s, regex_match, name_regex)) {
            dummy_attributes_.name = regex_match[1];
            ROS_INFO_STREAM("got node name: " << dummy_attributes_.name);
        }
    }

    /***************************
     *  Handle Orocos RT Type  *
     ***************************/

    std::regex rt_type_regex("^rt_type=(.+$)");

    for (const auto s : argv) {
        std::smatch regex_match;
        if (std::regex_match(s, regex_match, rt_type_regex)) {
            dummy_attributes_.rt_type = regex_match[1];
            ROS_INFO_STREAM(
                "got orocos rt type: " << dummy_attributes_.rt_type);
        }
    }

    /**********************************************
     *  Handle if node is start point in rt loop  *
     **********************************************/

    std::regex is_start_regex("^is_first=(.+$)");

    for (const auto s : argv) {
        std::smatch regex_match;
        if (std::regex_match(s, regex_match, is_start_regex)) {
            dummy_attributes_.is_start = (regex_match[1] == "true");
            ROS_INFO_STREAM(
                "Got node as rt start point: " << dummy_attributes_.is_start);
        }
    }
};

void sigintHandler(int sig) {
    ROS_DEBUG("sigint handler called");
    node_ptr->unloadInRTRunner();
    ros::shutdown();
}

int main(int argc, char **argv) {

    // necessary because ros::init is absorging argv
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) {
        args.push_back(argv[i]);
    }

    ros::init(argc, argv, "RTDummy", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    node_ptr = std::make_unique<RTDummyNode>(nh);
    signal(SIGINT, sigintHandler);

    node_ptr->handleArgs(args);

    node_ptr->configure();
    node_ptr->loop();
    node_ptr->shutdown();

    return 0;
}  // end main()

