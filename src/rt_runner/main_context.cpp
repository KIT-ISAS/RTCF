#include "main_context.hpp"
#include "rtt/extras/SlaveActivity.hpp"

MainContext::MainContext(std::string const& name)
    : TaskContext(name) {}

bool MainContext::configureHook(){
    return true;
}

bool MainContext::startHook(){
  return true;
}

void MainContext::updateHook(){
    for ( RTT::extras::SlaveActivity* slave : slaves_ ) {
        slave->execute();
    }
}

void MainContext::stopHook() {
}

void MainContext::cleanupHook() {
  std::cout << "MainContext cleaning up !" <<std::endl;
}

void MainContext::setSlaves(std::vector<RTT::extras::SlaveActivity*> slaves) {
    slaves_ = slaves;
};

void MainContext::clearSlaves() { slaves_.clear(); };
