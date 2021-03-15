#include "rt_runner_node.hpp"

#include <ros/ros.h>
#include <rtt/os/main.h>
#include <signal.h>

#include <iostream>
#include <memory>
#include <vector>

#include "rt_runner.hpp"
#include "rtcf/rtcf_types.hpp"

RTRunnerNode::RTRunnerNode(const ros::NodeHandle &node_handle) : node_handle_(node_handle), is_shutdown_(false) {
    rt_runner_ = std::make_shared<RTRunner>();
};

RTRunnerNode::~RTRunnerNode() {
    if (!is_shutdown_) {
        shutdown();
    }
};

bool RTRunnerNode::configure() {
    if (!loadROSParameters()) {
        ROS_ERROR("Error while getting parameters");
        return false;
    }
    rt_runner_->configure(settings_);
    setupROSServices();
    return true;
};

void RTRunnerNode::shutdown() {
    // NOTE: This should only be called after all components were taken down by the unload service calls
    shutdownROSServices();
    rt_runner_->shutdown();
    is_shutdown_ = true;
};

void RTRunnerNode::loop() { ros::spin(); };

void RTRunnerNode::setupROSServices() {
    // load and unload services are always there
    loadOrocosComponentService_   = node_handle_.advertiseService("/rt_runner/load_orocos_component",
                                                                &RTRunnerNode::loadOrocosComponentCallback, this);
    unloadOrocosComponentService_ = node_handle_.advertiseService("/rt_runner/unload_orocos_component",
                                                                  &RTRunnerNode::unloadOrocosComponentCallback, this);

    // activate and deactivate service is only there in one mode
    if (settings_.mode == RTRunner::Mode::WAIT_FOR_TRIGGER) {
        activateRTLoopService_ =
            node_handle_.advertiseService("/rt_runner/activate_rt_loop", &RTRunnerNode::activateRTLoopCallback, this);
        deactivateRTLoopService_ = node_handle_.advertiseService("/rt_runner/deactivate_rt_loop",
                                                                 &RTRunnerNode::deactivateRTLoopCallback, this);
    }
};

void RTRunnerNode::shutdownROSServices() {
    // shut down all ROS services
    loadOrocosComponentService_.shutdown();
    unloadOrocosComponentService_.shutdown();
    activateRTLoopService_.shutdown();
    deactivateRTLoopService_.shutdown();
};

bool RTRunnerNode::loadROSParameters() {
    // get mode
    std::string mode_str = node_handle_.param("mode", std::string(""));
    RTRunner::Mode mode  = RTRunner::string2Mode(mode_str);
    if (mode == RTRunner::Mode::UNKNOWN) {
        ROS_ERROR("Unknown mode specified");
        return false;
    } else {
        settings_.mode = mode;
    }

    // get number of expected components if mode is WAIT_FOR_COMPONENT
    if (settings_.mode == RTRunner::Mode::WAIT_FOR_COMPONENTS) {
        if (node_handle_.hasParam("num_components_expected")) {
            int num_components_expected;
            node_handle_.getParam("num_components_expected", num_components_expected);
            if (num_components_expected <= 0) {
                ROS_ERROR("Expected number of components too small.");
                return false;
            }
            settings_.expected_num_components = (size_t)num_components_expected;
        } else {
            ROS_ERROR("Expected number of components not given");
            return false;
        }
    }

    // get whitelist and blacklist mappings
    node_handle_.getParam("ros_mapping_whitelist", settings_.ros_mapping_whitelist);
    node_handle_.getParam("ros_mapping_blacklist", settings_.ros_mapping_blacklist);

    // get frequency
    if (node_handle_.hasParam("frequency")) {
        double frequency;
        node_handle_.getParam("frequency", frequency);
        if (frequency > 0.0) {
            settings_.frequency = frequency;
        } else {
            ROS_ERROR("Given frequency is infeasible");
            return false;
        }
    } else {
        settings_.frequency = 1.0;
        ROS_WARN("No frequency given, assuming a default frequency of 1 Hz");
    }

    return true;
}

bool RTRunnerNode::loadOrocosComponentCallback(rtcf::LoadOrocosComponent::Request &req,
                                               rtcf::LoadOrocosComponent::Response &res) {
    LoadAttributes attr;

    attr.name = req.component_name.data;
    attr.ns   = req.ns.data;
    for (const auto &m : req.mappings) {
        Mapping mapping;
        mapping.from_topic = m.from_topic.data;
        mapping.to_topic   = m.to_topic.data;
        attr.mappings.push_back(mapping);
    }

    attr.rt_package = req.component_pkg.data;
    attr.rt_type    = req.component_type.data;

    attr.topics_ignore_for_graph = req.topics_ignore_for_graph.data;
    attr.is_first                = req.is_first.data;
    attr.is_sync                 = req.is_sync.data;

    ROS_DEBUG_STREAM("Load service got called with following information:" << attr);

    res.success.data = true;  // rt_runner_->loadOrocosComponent(rt_type, name, ns, topics_ignore_for_graph, is_first,
                              // is_sync, mappings);
    return true;
};

bool RTRunnerNode::unloadOrocosComponentCallback(rtcf::UnloadOrocosComponent::Request &req,
                                                 rtcf::UnloadOrocosComponent::Response &res) {
    UnloadAttributes attr;
    attr.name = req.component_name.data;
    attr.ns   = req.ns.data;

    ROS_DEBUG_STREAM("Unload service got called with following information:" << attr);

    res.success.data = true;  // rt_runner_->unloadOrocosComponent(name, ns);
    return true;
};

bool RTRunnerNode::activateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    (void)req;

    rt_runner_->activateRTLoop();

    res.success = true;
    return true;
};

bool RTRunnerNode::deactivateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    (void)req;

    rt_runner_->deactivateRTLoop();

    res.success = true;
    return true;
};

void sigintHandler(int sig) {
    (void)sig;
    ROS_DEBUG("SIGINT handler called");
    // TODO: wait for all components to stop so that we can exit gracefully
    ROS_WARN("WAIT WOULD BE PLACED HERE!");

    node_ptr->shutdown();
    ros::shutdown();
}

int ORO_main(int argc, char **argv) {
    ros::init(argc, argv, "rt_runner", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    node_ptr = std::make_unique<RTRunnerNode>(nh);
    signal(SIGINT, sigintHandler);

    if (!node_ptr->configure()) {
        return EXIT_FAILURE;
    }

    node_ptr->loop();

    return EXIT_SUCCESS;
}
