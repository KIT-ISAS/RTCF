#include "main_context.hpp"

MainContext::MainContext(std::string const& name) : TaskContext(name) {}

bool MainContext::configureHook() {
    ROS_INFO("MainContext::configureHook() called");

    time_service_ptr_ = RTT::os::TimeService::Instance();
    time_service_ptr_->enableSystemClock(true);
    return true;
}

bool MainContext::startHook() {
    ROS_INFO("MainContext::startHook() called");
    return true;
}

void MainContext::updateHook() {
    // this does all the heavy lifting and calls all our components in order
    ROS_INFO("MainContext::updateHook() called");
    for (const auto& slave : slaves_) {
        slave->task_context->update(); // calls update on underlying activity
    }
}

void MainContext::stopHook() {
    ROS_INFO("MainContext::stopHook() called");
    // std::cout << "Main Context stopping !" << std::endl;
}

void MainContext::cleanupHook() {
    // std::cout << "MainContext cleaning up !" <<std::endl;
}

void MainContext::setSlaves(const RTOrder& slaves) {
    // std::cout << "Set slaves" << std::endl;
    slaves_ = slaves;
};

void MainContext::clearSlaves() {
    // std::cout << "Clear slaves" << std::endl;
    slaves_.clear();
};
