#ifndef RT_DYNAMIC_RECONFIGURE_H
#define RT_DYNAMIC_RECONFIGURE_H

#include <dynamic_reconfigure/server.h>

#include "rtcf/macros.hpp"
OROCOS_HEADERS_BEGIN
#include <rtt/base/DataObjectLockFree.hpp>
OROCOS_HEADERS_END

#include <thread>

/**
 * @brief This class providers a real-time safe wrapper for ROS dynamic reconfigure.
 * This is done using a non-blocking buffer provided by OROCOS.
 * @tparam A parameter struct with following properties:
 * - A typedef called ConfigType must exists and refer to a dynamic reconfigure type (e.g., package::SomenameConfig).
 * - The constructor must accept such a ConfigType object and store its values of interest in the struct.
 * - No allocations must take place in the constructor!
 */
template <class T>
class RTDynamicReconfigure {
    using ConfigType = typename T::ConfigType;

  public:
    RTDynamicReconfigure() : rt_data_(ConfigType(), 2) {}

    // We could also handle this in the constructor but this requires a NodeHandle, which is not available at
    // construction-time of the OROCOS component. Therefore, the RTDynamicReconfigure object must be initialized in
    // configureHook(), which is more convenient using a configure() method.
    void configure(const ros::NodeHandle& nh) {
        // create a server and link it to the callback
        server_ = std::make_shared<dynamic_reconfigure::Server<ConfigType>>(nh);
        server_->setCallback(boost::bind(&RTDynamicReconfigure::serverCallback, this, _1, _2));
        // this will also trigger the callback once, so no additional initialization needed
    }

    void getValue(T& data) { rt_data_.Get(data); }

  private:
    void serverCallback(ConfigType& config, uint32_t level) {
        (void)level;
        do {
            // do nothing
            // This is required since rt_data might fail during read from concurrent threads
            // In this case, we just repeat the read once more.
            // This is not problematic as we are in not in the real-time thread here.
        } while (!rt_data_.Set(T(config)));
    }

    std::shared_ptr<dynamic_reconfigure::Server<ConfigType>> server_;

    RTT::base::DataObjectLockFree<T> rt_data_;  // this buffer is real-time safe, as long as its contents are
};

#endif  // RT_DYNAMIC_RECONFIGURE_H
