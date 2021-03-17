#include "main_context.hpp"

#include "rtt/extras/SlaveActivity.hpp"

MainContext::MainContext(std::string const& name) : TaskContext(name) {}

bool MainContext::configureHook() {
    // std::cout << "Main context configured !" << std::endl;

    time_service_ptr_ = RTT::os::TimeService::Instance();
    time_service_ptr_->enableSystemClock(true);
    return true;
}

bool MainContext::startHook() { return true; }

void MainContext::updateHook() {
    // this does all the heavy lifting and calls all our components in order
    for (RTT::extras::SlaveActivity* slave : slaves_) {
        slave->execute();
    }
}

void MainContext::stopHook() {
    // std::cout << "Main Context stopping !" << std::endl;
}

void MainContext::cleanupHook() {
    // std::cout << "MainContext cleaning up !" <<std::endl;
}

void MainContext::setSlaves(SlaveActivityVector slaves) {
    // std::cout << "Set slaves" << std::endl;
    slaves_ = slaves;
};

void MainContext::clearSlaves() {
    // std::cout << "Clear slaves" << std::endl;
    slaves_.clear();
};
