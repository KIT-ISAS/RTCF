#ifndef RTCF_EXTENSION_H
#define RTCF_EXTENSION_H

#include <ros/ros.h>

class RtcfExtension {
    // allow some classes access to private parameters
    // reason: public setter methods are tempting for the component-developers
    friend class ComponentContainer;
    friend class MainContext;
    friend class RTRunner;

  private:
    // by using pointers the user will not accidentally use the node handle in the component constructor
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr nh_private_;

    inline static double frequency_;
    inline static double period_;
    inline static ros::Time last_timestamp_;

  public:
    RtcfExtension() { last_timestamp_ = ros::Time(0); };
    virtual ~RtcfExtension(){};

    const ros::NodeHandle& getNodeHandle() const { return *nh_; }
    const ros::NodeHandle& getPrivateNodeHandle() const { return *nh_private_; }

    const ros::Time& getTime() const { return last_timestamp_; }
    const double& getFrequency() const { return frequency_; }
    const double& getPeriod() const { return period_; }
};

#endif /* RTCF_EXTENSION_H */
