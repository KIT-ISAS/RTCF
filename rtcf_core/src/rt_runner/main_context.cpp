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

void MainContext::updateHook(){
    ticks_array_start[iteration_counter] = time_service_ptr->getNSecs();


    for ( RTT::extras::SlaveActivity* slave : slaves_ ) {
        slave->execute();
    }

    ticks_array_stop[iteration_counter] = time_service_ptr->getNSecs();
    iteration_counter++;

}

void MainContext::stopHook() {
    ROS_INFO_STREAM(iteration_counter << " lines in log");
    for (unsigned long long i = 0; i < iteration_counter; i++) {
        ROS_INFO_STREAM("start time:" << ticks_array_start[i]);
        ROS_INFO_STREAM("stop time:" << ticks_array_stop[i]);
    }
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
