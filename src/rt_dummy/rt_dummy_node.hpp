#ifndef RT_DUMMY_NODE_H
#define RT_DUMMY_NODE_H

#include "ros/ros.h"

#include "ros/service_client.h"
#include "rtcf/ActivateRTLoop.h"
#include "rtcf/DeactivateRTLoop.h"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/UnloadOrocosComponent.h"

class RTDummyNode {
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

    void loadInRTRunner();
    void unloadInRTRunner();

    void handleRemapping(std::vector<std::string> argv);

    ros::ServiceClient loadInRTRunnerClient;
    ros::ServiceClient unloadInRTRunnerClient;

    std::vector<std::pair<std::string, std::string>> mappings;
};

#endif /* RT_DUMMY_NODE_H */
