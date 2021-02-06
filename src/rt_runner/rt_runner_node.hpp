#ifndef RT_RUNNER_NODE_H
#define RT_RUNNER_NODE_H
#include "rt_runner.hpp"
#include <memory>

#include "ros/ros.h"

#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/UnloadOrocosComponent.h"
#include "rtcf/ActivateRTLoop.h"
#include "rtcf/DeactivateRTLoop.h"

class RTRunnerNode
{
private:
    ros::NodeHandle n;

public:
    RTRunnerNode(const ros::NodeHandle &node_handle);
    virtual ~RTRunnerNode();

    void configure();
    void shutdown();
    int loop();

    void setupROSServices();
    void shutdownROSServices();
    void loadROSParameters();

    bool loadOrocosComponentCallback(rtcf::LoadOrocosComponent::Request &req,
                                     rtcf::LoadOrocosComponent::Response &res);

    bool unloadOrocosComponentCallback(
        rtcf::UnloadOrocosComponent::Request &req,
        rtcf::UnloadOrocosComponent::Response &res);

    bool activateRTLoopCallback(rtcf::ActivateRTLoop::Request &req,
                                rtcf::ActivateRTLoop::Response &res);

    bool deactivateRTLoopCallback(rtcf::DeactivateRTLoop::Request &req,
                                  rtcf::DeactivateRTLoop::Response &res);

    ros::NodeHandle node_handle_;
    std::shared_ptr<RTRunner> rt_runner_;

    ros::ServiceServer loadOrocosComponentService;
    ros::ServiceServer unloadOrocosComponentService;
    ros::ServiceServer activateRTLoopService;
    ros::ServiceServer deactivateRTLoopService;

};

#endif /* RT_RUNNER_NODE_H */   
