#include "component.hpp"

#include <iostream>
#include <rtt/Component.hpp>

SumTest::SumTest(std::string const& name) : TaskContext(name), out_("out"), in1_("in1"), in2_("in2") {
    std::cout << "SumTest constructed !" << std::endl;
}

bool SumTest::configureHook() {
    std::cout << "SumTest configured !" << std::endl;

    this->ports()->addPort(out_);
    this->ports()->addPort(in1_);
    this->ports()->addPort(in2_);
    return true;
}

bool SumTest::startHook() {
    std::cout << "SumTest started !" << std::endl;
    return true;
}

void SumTest::updateHook() {
    std::cout << "SumTest executes updateHook !" << std::endl;

    // read will copy old data, which is fine
    bool data_avail = true;
    data_avail &= (in1_.read(in_msg_1_) >= RTT::OldData);
    data_avail &= (in2_.read(in_msg_2_) >= RTT::OldData);

    if (data_avail) {
        out_msg_.data = in_msg_1_.data + in_msg_2_.data;
        out_.write(out_msg_);
    }
}

void SumTest::stopHook() { std::cout << "SumTest executes stopping !" << std::endl; }

void SumTest::cleanupHook() { std::cout << "SumTest cleaning up !" << std::endl; }

ORO_CREATE_COMPONENT(SumTest)
