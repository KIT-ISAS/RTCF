#include "rt_runner.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <ocl/TaskBrowser.hpp>

#include <rtt/os/main.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

/**
* Include this header in order to use methods.
*/
#include <rtt/OperationCaller.hpp>

#include <ocl/OCL.hpp>

sender::sender(std::string const& name) : TaskContext(name, PreOperational){
  std::cout << "sender constructed !" <<std::endl;
}

bool sender::configureHook(){
    time_service_ptr=RTT::os::TimeService::Instance();
    time_service_ptr->enableSystemClock(true);
    this->ports()->addPort( "Port", Port ).doc( "Output Port, here write our data to." );
    return true;
}

bool sender::startHook(){
    //current_time = time_service_ptr->ticksGet();
    if ( !Port.connected() ) {
        return false;
    }
  return true;
}

void sender::updateHook(){
    ticks_array[iteration_counter] = time_service_ptr->getNSecs();
    iteration_counter++;
    Port.write(msg);
    //msg++;
}

void sender::stopHook() {
    Port.disconnect();
    RTT::Logger::In in("sender");
    RTT::log(RTT::LoggerLevel::Info) << iteration_counter << " lines in log" << RTT::endlog();
    for (unsigned long long i = 0; i < iteration_counter; i++) {
        RTT::log(RTT::LoggerLevel::Info)
            << ticks_array[i] << RTT::endlog();
    }
}

void sender::cleanupHook() {
  std::cout << "sender cleaning up !" <<std::endl;
}






reciever::reciever(std::string const& name) : TaskContext(name, PreOperational){
  std::cout << "reciever constructed !" <<std::endl;
}

bool reciever::configureHook(){
    time_service_ptr=RTT::os::TimeService::Instance();
    time_service_ptr->enableSystemClock(true);
    this->ports()->addPort( "Port", Port ).doc( "Input Port that raises an event." );
    return true;
}

bool reciever::startHook(){
    current_time = time_service_ptr->ticksGet();
    if ( !Port.connected() ) {
    return false;
    }
  return true;
}

void reciever::updateHook(){
    ticks_array[iteration_counter] = time_service_ptr->getNSecs();
    iteration_counter++;
    //if (Port.read(msg)==RTT::NewData) {
        //recieved_msgs++;
    //}

}

void reciever::stopHook() {
    RTT::Logger::In in("reciever");
    RTT::log(RTT::LoggerLevel::Info) << iteration_counter << " reciever" << RTT::endlog();
    RTT::log(RTT::LoggerLevel::Info) << recieved_msgs << " msgs recieved" << RTT::endlog();
    for (unsigned long long i = 0; i < iteration_counter; i++) {
        RTT::log(RTT::LoggerLevel::Info)
            << ticks_array[i] << RTT::endlog();
    }
}

void reciever::cleanupHook() {
  std::cout << "reciever cleaning up !" <<std::endl;
}


master::master(std::string const& name, RTT::extras::SlaveActivity* slave1,
               RTT::extras::SlaveActivity* slave2)
    : TaskContext(name, PreOperational) {

        slave_one = slave1;
        slave_two = slave2;
    std::cout << "master constructed !" << std::endl;
}


bool master::configureHook(){
    return true;
}

bool master::startHook(){
  return true;
}

void master::updateHook(){
    slave_one->execute();
    slave_two->execute();
}

void master::stopHook() {
}

void master::cleanupHook() {
  std::cout << "master cleaning up !" <<std::endl;
}


using namespace RTT;
int ORO_main(int argc, char** argv)
{
    Logger::In in("main()");

    // Set log level more verbose than default,
    // such that we can see output :
    if ( log().getLogLevel() < Logger::Info ) {
        log().setLogLevel( Logger::Info );
        log(Info) << argv[0] << " manually raises LogLevel to 'Info' (5). See also file 'orocos.log'."<<endlog();
    }

    RTT::Activity* master_one = new RTT::Activity(ORO_SCHED_RT, 99, 0.01 );
    // a 'slave', takes over properties (period,...) of 'master_one':
    RTT::extras::SlaveActivity* slave_one = new RTT::extras::SlaveActivity( master_one );
    RTT::extras::SlaveActivity* slave_two = new RTT::extras::SlaveActivity( master_one );


    // Create the task:
    sender sender("sender");
    reciever reciever("reciever");

    master master("master", slave_one, slave_two);

    // Create the activity which runs the task's engine:
    master.setActivity(master_one);
    sender.setActivity( slave_one );
    reciever.setActivity( slave_two );

    master.configure();
    sender.configure();
    reciever.configure();

    sender.connectPorts(&reciever);

    master.start();
    sender.start();
    reciever.start();

    log(Info) << "**** Starting the TaskBrowser       ****" <<endlog();
    // Switch to user-interactive mode.
    OCL::TaskBrowser browser( &sender );

    // Accept user commands from console.
    browser.loop();
    
    master.stop();
    sender.stop();
    reciever.stop();

    return 0;
}
