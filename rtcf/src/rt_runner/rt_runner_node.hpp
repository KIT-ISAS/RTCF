#ifndef RT_RUNNER_NODE_H
#define RT_RUNNER_NODE_H

#include <ros/ros.h>
#include <rtcf_msgs/GetRunnerInfo.h>
#include <rtcf_msgs/LoadOrocosComponent.h>
#include <rtcf_msgs/UnloadOrocosComponent.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <mutex>

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
    ros::ServiceServer introspectionService_;

    bool loadOrocosComponentCallback(rtcf_msgs::LoadOrocosComponent::Request &req,
                                     rtcf_msgs::LoadOrocosComponent::Response &res);
    bool unloadOrocosComponentCallback(rtcf_msgs::UnloadOrocosComponent::Request &req,
                                       rtcf_msgs::UnloadOrocosComponent::Response &res);

    bool activateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool deactivateRTLoopCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool introspectionCallback(rtcf_msgs::GetRunnerInfo::Request &req, rtcf_msgs::GetRunnerInfo::Response &res);

    void setupROSServices();
    void shutdownROSServices();

    bool loadROSParameters();

  public:
    RTRunnerNode(const ros::NodeHandle &node_handle);
    virtual ~RTRunnerNode();

    bool configure();
    void shutdown();
    void loop();

    std::mutex mtx_;
};

static std::unique_ptr<RTRunnerNode> node_ptr;

#endif /* RT_RUNNER_NODE_H */