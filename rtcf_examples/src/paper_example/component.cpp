#include <std_msgs/Float32.h>

#include <rtcf/macros.hpp>
#include <rtcf/rtcf_extension.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>
OROCOS_HEADERS_END

class PaperExample : public RTT::TaskContext, public RtcfExtension {
  public:
    PaperExample(std::string const& name) : TaskContext(name), port_out_("out_port"), port_in_("in_port") {}

    bool configureHook() {
        this->ports()->addPort(port_in_);
        this->ports()->addPort(port_out_);
        double param = this->getNodeHandle().param("/test_parameter", 0.0);
        NON_RT_INFO_STREAM("Fetched param with value " << param);
        return true;
    }
    void updateHook() {
        RT_INFO("Update hook called!");
        port_in_.read(msg_);
        port_out_.write(msg_);
    }

    bool startHook() { return true; }
    void stopHook() {}
    void cleanupHook() {}

  private:
    RTT::OutputPort<std_msgs::Float32> port_out_;
    RTT::InputPort<std_msgs::Float32> port_in_;
    std_msgs::Float32 msg_;
};

ORO_CREATE_COMPONENT(PaperExample)
