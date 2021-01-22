#ifndef RT_RUNNER_NODE_H
#define RT_RUNNER_NODE_H
#include "rt_runner.hpp"
#include <memory>

#include "ros/ros.h"

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
    
    void setupROS();
    void shutdownROS();
    void loadROSParameters();
    
    void loadOrocosComponentCallback();
    void unloadOrocosComponentCallback();
    void activateRTLoopCallback();
    void deactivateRTLoopCallback();

    ros::NodeHandle node_handle_;
    std::shared_ptr<RTRunner> rt_runner_;
};

#endif /* RT_RUNNER_NODE_H */   
