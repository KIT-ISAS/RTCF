#ifndef RT_LAUNCHER_NODE_H
#define RT_LAUNCHER_NODE_H

#include "ros/ros.h"
#include "ros/service_client.h"
#include "rtcf/LoadOrocosComponent.h"
#include "rtcf/UnloadOrocosComponent.h"
#include "rtcf/rtcf_types.hpp"

class RTLauncherNode {
  private:
    ros::NodeHandle node_handle_;

    LoadAttributes launcher_attributes_;

    rtcf::LoadOrocosComponent genLoadMsg();
    rtcf::UnloadOrocosComponent genUnloadMsg();

  public:
    RTLauncherNode(const ros::NodeHandle &node_handle);

    void configure();
    void shutdown();
    void loop();

    void setupServiceClients();
    void shutdownServiceClients();

    bool loadInRTRunner();
    bool unloadInRTRunner();

    bool handleArgs(int &argc, char **argv);
    void loadNodeConfiguration();
    bool loadROSParameters();

    ros::ServiceClient loadInRTRunnerClient;
    ros::ServiceClient unloadInRTRunnerClient;
};

static std::unique_ptr<RTLauncherNode> node_ptr;

#endif /* RT_LAUNCHER_NODE_H */
