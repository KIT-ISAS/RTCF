#ifndef RT_RUNNER_NODE_H
#define RT_RUNNER_NODE_H
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <rtcf/LoadOrocosComponent.h>
#include <rtcf/UnloadOrocosComponent.h>
#include <std_srvs/Trigger.h>

#include <memory>

#include "ros/ros.h"
#include "rt_runner.hpp"

class RTRunnerNode {
  private:
    ros::NodeHandle node_handle_;
    std::shared_ptr<RTRunner> rt_runner_;
    RTRunner::Settings settings_;
    bool is_shutdown_;

    ros::ServiceServer loadOrocosComponentService_;
    ros::ServiceServer unloadOrocosComponentService_;
    ros::ServiceServer activateRTLoopService_;
    ros::ServiceServer deactivateRTLoopService_;

    bool loadOrocosComponentCallback(rtcf::LoadOrocosComponent::Request &req, rtcf::LoadOrocosComponent::Response &res);
    bool unloadOrocosComponentCallback(rtcf::UnloadOrocosComponent::Request &req,
                                       rtcf::UnloadOrocosComponent::Response &res);

    bool activateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool deactivateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void setupROSServices();
    void shutdownROSServices();

    bool loadROSParameters();

  public:
    RTRunnerNode(const ros::NodeHandle &node_handle);
    virtual ~RTRunnerNode();

    bool configure();
    void shutdown();
    void loop();
};

static std::unique_ptr<RTRunnerNode> node_ptr;

#endif /* RT_RUNNER_NODE_H */
