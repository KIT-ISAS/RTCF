#ifndef RT_DUMMY_NODE_H
#define RT_DUMMY_NODE_H

#include "ros/ros.h"

class RTDummyNode
{
private:
    ros::NodeHandle n;
    
    ros::NodeHandle node_handle_;

public:
    RTDummyNode(const ros::NodeHandle &node_handle);

    void configure();
    void shutdown();
    int loop();
    
    void setupROS();
    void shutdownROS();
    void loadROSParameters();

    void registerAtRTRunner();
    void unregisterAtRTRunner();

    void handleRemapping(int argc, char **argv);
};

#endif /* RT_DUMMY_NODE_H */
