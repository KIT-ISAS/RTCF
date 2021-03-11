#ifndef RT_LAUNCHER_NODE_H
#define RT_LAUNCHER_NODE_H

#include "ros/ros.h"
#include "ros/service_client.h"
#include "rtcf/ActivateRTLoop.h"
#include "rtcf/DeactivateRTLoop.h"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/UnloadOrocosComponent.h"

struct Mapping {
    std::string from_topic;
    std::string to_topic;
};

struct LauncherAttributes {
    std::vector<Mapping> mappings;
    std::string name;
    std::string rt_type;
    std::string topics_ignore_for_graph;
    bool is_start = false;
    bool is_sync  = false;
};

class RTLauncherNode {
  private:
    ros::NodeHandle node_handle_;

    LauncherAttributes launcher_attributes_;

    rtcf::LoadOrocosComponent genLoadMsg();
    rtcf::UnloadOrocosComponent genUnloadMsg();

  public:
    RTLauncherNode(const ros::NodeHandle &node_handle);

    void configure();
    void shutdown();
    int loop();

    void setupServiceClients();
    void shutdownServiceClients();
    void loadROSParameters();

    void loadInRTRunner();
    void unloadInRTRunner();

    void handleArgs(std::vector<std::string> argv);

    ros::ServiceClient loadInRTRunnerClient;
    ros::ServiceClient unloadInRTRunnerClient;
};

static std::unique_ptr<RTLauncherNode> node_ptr;

#endif /* RT_LAUNCHER_NODE_H */
