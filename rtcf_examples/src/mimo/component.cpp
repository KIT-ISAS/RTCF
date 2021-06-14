#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

#include <rtcf/rt_logging_macros.hpp>

Mimo::Mimo(std::string const& name) :
    TaskContext(name),
    port_out1_("out1"),
    port_out2_("out2"),
    port_in1_("in1"),
    port_in2_("in2"),
    port_in3_("in3"),
    port_in4_("in4") {
    NON_RT_INFO("Mimo constructed !");
}

bool Mimo::configureHook() {
    NON_RT_INFO("Mimo configured !");

    this->ports()->addPort(port_in1_);
    this->ports()->addPort(port_in2_);
    this->ports()->addPort(port_in3_);
    this->ports()->addPort(port_in4_);

    this->ports()->addPort(port_out1_);
    this->ports()->addPort(port_out2_);

    return true;
}

bool Mimo::startHook() {
    NON_RT_INFO("Mimo started !");
    return true;
}

void Mimo::updateHook() {
    // read will copy old data, which is fine
    std_msgs::Float64 in_msg_1_, in_msg_2_, in_msg_3_, in_msg_4_;
    in_msg_1_.data = 1.0;
    port_in1_.read(in_msg_1_, false);
    port_in2_.read(in_msg_2_, false);
    port_in3_.read(in_msg_3_, false);
    port_in4_.read(in_msg_4_, false);

    std_msgs::Float64 out_msg_;
    // just do a plain simple addition with some overflow checking
    out_msg_.data = in_msg_1_.data + in_msg_2_.data + in_msg_3_.data + in_msg_4_.data;
    // make sure float does not overflow (pure addition will cause an overflow)
    if (out_msg_.data > 1e20) {
        out_msg_.data = 1;
    }
    port_out1_.write(out_msg_);
    port_out2_.write(out_msg_);
}

void Mimo::stopHook() { NON_RT_INFO("Mimo executes stopping !"); }

void Mimo::cleanupHook() { NON_RT_INFO("Mimo cleaning up !"); }

ORO_CREATE_COMPONENT(Mimo)
