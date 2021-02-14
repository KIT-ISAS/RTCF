#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include "ros/ros.h"

class RtcfExtension
{
private:

public:
    RtcfExtension() {};
    virtual ~RtcfExtension() {};

    ros::NodeHandle* node_handle_ptr_;
};

#endif /* RTCF_EXTENSION_H */
