
#include <ros/ros.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/os/main.h>
#include <rtt_ros/rtt_ros.h>
OROCOS_HEADERS_END

#include <signal.h>

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <memory>
#include <vector>

#include "rt_runner.hpp"
#include "rt_runner_node.hpp"
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
    const std::lock_guard<std::mutex> lock(mtx_);
    if (!loadROSParameters()) {
        ROS_ERROR("Error while getting parameters");
        return false;
    }
    rt_runner_->configure(settings_);
    setupROSServices();
    return true;
}

void RTRunnerNode::shutdown() {
    {
        const std::lock_guard<std::mutex> lock(mtx_);
        rt_runner_->shutdown();  // this stops the execution engine of all components
    }
    ROS_INFO("Waiting for components to be unloaded...");
    // for simplicity, use polling here
    // TODO: (optional) use conditional variable or something similar for waiting
    while (rt_runner_->getNumLoadedComponents() > 0) {
        ros::WallDuration(0.1).sleep();
    }
    shutdownROSServices();
    rt_runner_->finalize();
};

void RTRunnerNode::loop() {
    // NOTE: This is somewhat special due to OROCOS design!
    // When a component a RTT component living in a ROS package depends on any ROS messages that have been converted to
    // an OROCOS typekit, rtt_roscomm and rtt_rosnode will be loaded automatically as dependency. This will cause the
    // instatiation of a ros::AsyncSpinner that will get into conflict with the single threaded ros::spin(). For this
    // reason, we preemptively import the rtt_roscomm component and use its spinner here for doing the ROS service
    // calls. Hence, we do not need to spin separately, but just need to wait. To ensure thread safety, a mutex is used
    // in critical sections of this class.
    ros::waitForShutdown();
};

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
    boost::algorithm::to_lower(mode_str);
    RTRunner::Mode mode;
    if (mode_str == "") {
        ROS_WARN("Using default mode (NO_WAIT)");
        mode = RTRunner::Mode::NO_WAIT;
    } else {
        mode = RTRunner::string2Mode(mode_str);
        if (mode == RTRunner::Mode::UNKNOWN) {
            ROS_ERROR("Unknown mode specified");
            return false;
        }
    }
    settings_.mode = mode;

    // get wait policy
    std::string wait_policy_str = node_handle_.param("wait_policy", std::string("absolute"));
    boost::algorithm::to_lower(wait_policy_str);
    RTRunner::WaitPolicy policy;
    policy = RTRunner::string2WaitPolicy(wait_policy_str);
    if (policy == RTRunner::WaitPolicy::UNKNOWN) {
        ROS_ERROR("Unknown wait policy specified");
        return false;
    }
    settings_.wait_policy = policy;

    // get number of expected components if mode is WAIT_FOR_COMPONENT
    if (settings_.mode == RTRunner::Mode::WAIT_FOR_COMPONENTS) {
        if (node_handle_.hasParam("num_components_expected")) {
            int num_components_expected;
            node_handle_.getParam("num_components_expected", num_components_expected);
            if (num_components_expected <= 0) {
                ROS_ERROR("Expected number of components too small");
                return false;
            }
            settings_.expected_num_components = (size_t)num_components_expected;
        } else {
            ROS_ERROR("Expected number of components not given, but required");
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

    // simulation time
    settings_.is_simulation = node_handle_.param("/use_sim_time", false);

    return true;
}

bool RTRunnerNode::loadOrocosComponentCallback(rtcf_msgs::LoadOrocosComponent::Request &req,
                                               rtcf_msgs::LoadOrocosComponent::Response &res) {
    const std::lock_guard<std::mutex> lock(mtx_);

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

    ROS_DEBUG_STREAM("Load service got called with following information:" << std::endl << attr);

    res.success.data = rt_runner_->loadOrocosComponent(attr);
    return true;
};

bool RTRunnerNode::unloadOrocosComponentCallback(rtcf_msgs::UnloadOrocosComponent::Request &req,
                                                 rtcf_msgs::UnloadOrocosComponent::Response &res) {
    const std::lock_guard<std::mutex> lock(mtx_);

    UnloadAttributes attr;
    attr.name = req.component_name.data;
    attr.ns   = req.ns.data;

    ROS_DEBUG_STREAM("Unload service got called with following information:" << std::endl << attr);

    res.success.data = rt_runner_->unloadOrocosComponent(attr);
    return true;
};

bool RTRunnerNode::activateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    (void)req;

    const std::lock_guard<std::mutex> lock(mtx_);
    rt_runner_->activateTrigger();

    res.success = true;
    return true;
};

bool RTRunnerNode::deactivateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    (void)req;

    const std::lock_guard<std::mutex> lock(mtx_);
    rt_runner_->deactivateTrigger();

    res.success = true;
    return true;
};

void sigintHandler(int sig) {
    (void)sig;
    ROS_DEBUG("SIGINT handler called");
    node_ptr->shutdown();
    ros::shutdown();
}

int ORO_main(int argc, char **argv) {
    ros::init(argc, argv, "rt_runner", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    // ros::Duration(8.0).sleep(); // for debugging

    node_ptr = std::make_unique<RTRunnerNode>(nh);
    signal(SIGINT, sigintHandler);

    if (!node_ptr->configure()) {
        return EXIT_FAILURE;
    }

    // this will indirectly provide an asyncronous spinner
    if (!rtt_ros::import("rtt_roscomm")) {
        ROS_ERROR("Could not load rtt_roscomm component");
        return EXIT_FAILURE;
    }
    node_ptr->loop();

    return EXIT_SUCCESS;
}
