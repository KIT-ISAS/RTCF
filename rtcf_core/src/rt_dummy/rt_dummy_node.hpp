#ifndef RT_DUMMY_NODE_H
#define RT_DUMMY_NODE_H

#include "ros/ros.h"

#include "ros/service_client.h"
#include "rtcf/ActivateRTLoop.h"
#include "rtcf/DeactivateRTLoop.h"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/UnloadOrocosComponent.h"

struct mapping {
    std::string from_topic;
    std::string to_topic;
};

struct dummy_attributes {
    std::vector<mapping> mappings;
    std::string name;
    std::string rt_type;
    bool is_start = false;
};

class RTDummyNode {
   private:
    ros::NodeHandle node_handle_;

    dummy_attributes dummy_attributes_;

    rtcf::LoadOrocosComponent genLoadMsg();
    rtcf::UnloadOrocosComponent genUnloadMsg();

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

    void handleArgs(std::vector<std::string> argv);

    ros::ServiceClient loadInRTRunnerClient;
    ros::ServiceClient unloadInRTRunnerClient;

};

static std::unique_ptr<RTDummyNode> node_ptr;

#endif /* RT_DUMMY_NODE_H */
