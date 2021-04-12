#ifndef RT_DYNAMIC_RECONFIGURE_H
#define RT_DYNAMIC_RECONFIGURE_H

#include <dynamic_reconfigure/server.h>

#include <rtt/base/DataObjectLockFree.hpp>

/**
 * @brief
 * @tparam T Generated dynamic reconfigure type (e.g., package::SomenameConfig)
 */
template <class T>
class RTDynamicReconfigure {
  public:
    RTDynamicReconfigure() : rt_data_(T(), 2) {}

    // We could also handle this in the constructor but this requires a NodeHandle, which is not available at
    // construction-time of the OROCOS component. Therefore, the RTDynamicReconfigure object must be initialized in
    // configureHook(), which is more convenient using a configure() method.
    void configure(const ros::NodeHandle& nh) {
        // create a server and link it to the callback
        server_ = std::make_shared<dynamic_reconfigure::Server<T>>(nh);
        server_->setCallback(boost::bind(&RTDynamicReconfigure::serverCallback, this, _1, _2));
        // this will also trigger the callback once, so no additional initialization needed
    }

    void getValue(T& data) { rt_data_.Get(data); }

  private:
    void serverCallback(T& config, uint32_t level) {
        (void)level;
        rt_data_.Set(config);
    }

    std::shared_ptr<dynamic_reconfigure::Server<T>> server_;
    RTT::base::DataObjectLockFree<T> rt_data_;
};

#endif  // RT_DYNAMIC_RECONFIGURE_H
