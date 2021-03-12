#include "rt_launcher_node.hpp"

#include <signal.h>

#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "ros/duration.h"
#include "ros/service.h"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/Mapping.h"

RTLauncherNode::RTLauncherNode(const ros::NodeHandle &node_handle) : node_handle_(node_handle){};

int RTLauncherNode::loop() {
    ros::spin();
    return 0;
};

void RTLauncherNode::configure() { setupServiceClients(); };

void RTLauncherNode::shutdown() { shutdownServiceClients(); };

void RTLauncherNode::setupServiceClients() {
    ros::service::waitForService("/rt_runner/load_orocos_component", ros::Duration(-1));
    loadInRTRunnerClient = node_handle_.serviceClient<rtcf::LoadOrocosComponent>("/rt_runner/load_orocos_component");

    ros::service::waitForService("/rt_runner/unload_orocos_component", ros::Duration(-1));
    unloadInRTRunnerClient =
        node_handle_.serviceClient<rtcf::UnloadOrocosComponent>("/rt_runner/unload_orocos_component");
};

void RTLauncherNode::shutdownServiceClients() {
    loadInRTRunnerClient.shutdown();
    unloadInRTRunnerClient.shutdown();
};

rtcf::LoadOrocosComponent RTLauncherNode::genLoadMsg() {
    rtcf::LoadOrocosComponent srv;

    for (auto mapping : launcher_attributes_.mappings) {
        rtcf::Mapping m;

        m.from_topic.data = mapping.from_topic;
        m.to_topic.data   = mapping.to_topic;

        srv.request.mappings.push_back(m);
    }

    srv.request.component_name.data          = launcher_attributes_.name;
    srv.request.component_type.data          = launcher_attributes_.rt_type;
    srv.request.is_start.data                = launcher_attributes_.is_start;
    srv.request.is_sync.data                 = launcher_attributes_.is_sync;
    srv.request.topics_ignore_for_graph.data = launcher_attributes_.topics_ignore_for_graph;

    srv.request.ns.data = node_handle_.getNamespace();

    return srv;
};

rtcf::UnloadOrocosComponent RTLauncherNode::genUnloadMsg() {
    rtcf::UnloadOrocosComponent srv;

    srv.request.component_name.data = launcher_attributes_.name;

    std::stringstream ss;
    ss << node_handle_.getNamespace();
    srv.request.ns.data = ss.str();

    return srv;
};

void RTLauncherNode::loadROSParameters() {
    bool is_first = false;
    if (node_handle_.getParam("is_first", is_first)) {
        launcher_attributes_.is_start = is_first;
    }

    std::string rt_type;
    if (node_handle_.getParam("rt_type", rt_type)) {
        launcher_attributes_.rt_type = rt_type;
    } else {
        ROS_ERROR_STREAM("No rt_type for RT launcher given");
    }

    bool is_sync = false;
    if (node_handle_.getParam("is_sync", is_sync)) {
        launcher_attributes_.is_sync = is_sync;
    }

    std::string topics_ignore_for_graph;
    if (node_handle_.getParam("topics_ignore_for_graph", topics_ignore_for_graph)) {
        launcher_attributes_.topics_ignore_for_graph = topics_ignore_for_graph;
    }
}

bool RTLauncherNode::loadInRTRunner() {
    rtcf::LoadOrocosComponent srv = genLoadMsg();

    bool service_ok = loadInRTRunnerClient.call(srv);
    if (service_ok && srv.res.success) {
        ROS_DEBUG("RT Runner load called successfully");
        return true;
    } else {
        ROS_ERROR("Failed to call load service in RT Runner");
        return false;
    }
};

bool RTLauncherNode::unloadInRTRunner() {
    rtcf::UnloadOrocosComponent srv = genUnloadMsg();

    bool service_ok = unloadInRTRunnerClient.call(srv);
    if (service_ok && srv.res.success) {
        ROS_DEBUG("RT Runner unload called successfully");
        return true;
    } else {
        ROS_ERROR("Failed to call unload service in RT Runner");
        return false;
    }
};

void RTLauncherNode::handleArgs(std::vector<std::string> argv) {
    // Alternative to using regexes is boost::program_options
    // but for now it works :-)

    /*****************************
     *  Handle Topic Remappings  *
     *****************************/

    std::regex from_regex("(^[a-zA-Z0-9\\/].+):=[a-zA-Z0-9\\/].+$");
    std::regex to_regex("^[a-zA-Z0-9\\/].+:=([a-zA-Z0-9\\/].+$)");

    for (const auto s : argv) {
        std::smatch from_regex_match;
        std::smatch to_regex_match;

        bool from_match_found;
        bool to_match_found;

        from_match_found = std::regex_match(s, from_regex_match, from_regex);
        to_match_found   = std::regex_match(s, to_regex_match, to_regex);

        if (from_match_found && to_match_found) {
            Mapping mapping;

            mapping.from_topic = from_regex_match[1];
            mapping.to_topic   = to_regex_match[1];

            launcher_attributes_.mappings.push_back(mapping);

            ROS_DEBUG_STREAM("Got remapping from: [" << mapping.from_topic << "] to: [" << mapping.to_topic << "]");
        }
    }

    /**********************
     *  Handle Node Name  *
     **********************/

    std::regex name_regex("^__name:=(.+$)");

    for (const auto s : argv) {
        std::smatch regex_match;
        if (std::regex_match(s, regex_match, name_regex)) {
            launcher_attributes_.name = regex_match[1];
            ROS_DEBUG_STREAM("Got node name: " << launcher_attributes_.name);
        }
    }
};

void sigintHandler(int sig) {
    ROS_DEBUG("SIGINT handler called");
    node_ptr->unloadInRTRunner();
    node_ptr->shutdown();
    // after ros::shutdown is called, ros::spin() will return
    ros::shutdown();
}

int main(int argc, char **argv) {
    // necessary because ros::init is absorbing argv
    std::vector<std::string> args;
    for (int i = 0; i < argc; i++) {
        args.push_back(argv[i]);
    }

    ros::init(argc, argv, "RTLauncher", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    node_ptr = std::make_unique<RTLauncherNode>(nh);
    signal(SIGINT, sigintHandler);

    node_ptr->handleArgs(args);

    node_ptr->configure();
    node_ptr->loadROSParameters();
    node_ptr->loadInRTRunner();
    node_ptr->loop();

    return 0;
}  // end main()
