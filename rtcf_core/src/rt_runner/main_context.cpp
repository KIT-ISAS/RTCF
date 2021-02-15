#include "main_context.hpp"
#include "rtt/extras/SlaveActivity.hpp"

MainContext::MainContext(std::string const& name)
    : TaskContext(name) {}

bool MainContext::configureHook(){
    std::cout << "Main context configured !" << std::endl;

    time_service_ptr = RTT::os::TimeService::Instance();
    time_service_ptr->enableSystemClock(true);
    return true;
}

bool MainContext::startHook(){
  return true;
}

void MainContext::updateHook() {
    for (RTT::extras::SlaveActivity* slave : slaves_) {
        slave->execute();
    }
}

void MainContext::stopHook() {
    std::cout << "Main Context stopping !" << std::endl;
}

void MainContext::cleanupHook() {
  std::cout << "MainContext cleaning up !" <<std::endl;
}

void MainContext::setSlaves(std::vector<RTT::extras::SlaveActivity*> slaves) {
    std::cout << "Set Slaves" << std::endl;
    slaves_ = slaves;
};

void MainContext::clearSlaves() { slaves_.clear(); };
