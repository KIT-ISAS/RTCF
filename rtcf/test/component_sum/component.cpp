#include "component.hpp"

#include <rtcf/macros.hpp>
OROCOS_HEADERS_BEGIN
#include <rtt/Component.hpp>
OROCOS_HEADERS_END

#include <rtcf/rt_logging_macros.hpp>

SumTest::SumTest(std::string const& name) : TaskContext(name), out_("out"), in1_("in1"), in2_("in2") {
    NON_RT_INFO("SumTest constructed !");
}

bool SumTest::configureHook() {
    NON_RT_INFO("SumTest configured !");

    this->ports()->addPort(out_);
    this->ports()->addPort(in1_);
    this->ports()->addPort(in2_);
    return true;
}

bool SumTest::startHook() {
    NON_RT_INFO("SumTest started !");
    return true;
}

void SumTest::updateHook() {
    // read will copy old data, which is fine
    bool data_avail = true;
    data_avail &= (in1_.read(in_msg_1_) >= RTT::OldData);
    data_avail &= (in2_.read(in_msg_2_) >= RTT::OldData);

    if (data_avail) {
        out_msg_.data = in_msg_1_.data + in_msg_2_.data;
        out_.write(out_msg_);
    }
}

void SumTest::stopHook() { NON_RT_INFO("SumTest executes stopping !"); }

void SumTest::cleanupHook() { NON_RT_INFO("SumTest cleaning up !"); }

ORO_CREATE_COMPONENT(SumTest)
