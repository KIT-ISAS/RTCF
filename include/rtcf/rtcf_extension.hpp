#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include "ros/ros.h"

class RtcfExtension
{
private:

public:
    RtcfExtension();
    virtual ~RtcfExtension();

    ros::NodeHandle nh_;
};

#endif /* RTCF_EXTENSION_H */
