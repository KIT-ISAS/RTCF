#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include <ros/ros.h>

class RtcfExtension {
    friend class ComponentContainer;  // allow ComponentContainer access to private parameters

  private:
    // by using pointers the user will not accidentally use the node handle in the component constructor
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr nh_private_;

    // static double frequency;
    // static ros::Time last_timestamp;

  public:
    RtcfExtension(){};
    virtual ~RtcfExtension(){};

    ros::NodeHandle& getNodeHandle() const { return *nh_; }
    ros::NodeHandle& getPrivateNodeHandle() const { return *nh_private_; }
};

#endif /* RTCF_EXTENSION_H */
