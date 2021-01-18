#ifndef OROCOS_SLAVE_ACTIVITY_NON_RT_COMPONENT_HPP
#define OROCOS_SLAVE_ACTIVITY_NON_RT_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/TimeService.hpp>

class sender  : public RTT::TaskContext{
  public:
    sender(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::os::TimeService* time_service_ptr;
    RTT::os::TimeService::ticks current_time;

    RTT::os::TimeService::nsecs ticks_array[500000];
    unsigned long long iteration_counter = 0;

    RTT::OutputPort<double> Port;

    bool msg = true;
};

class reciever : public RTT::TaskContext{
  public:
    reciever(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::os::TimeService* time_service_ptr;
    RTT::os::TimeService::ticks current_time;

    RTT::os::TimeService::nsecs ticks_array[500000];
    unsigned long long iteration_counter = 0;

    RTT::InputPort<double> Port;

    double msg = 0;
    
    double recieved_msgs = 0;
};


class master : public RTT::TaskContext{
  public:
    master(std::string const& name, RTT::extras::SlaveActivity* slave1, RTT::extras::SlaveActivity* slave2);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    RTT::extras::SlaveActivity* slave_one;
    RTT::extras::SlaveActivity* slave_two;

};


#endif
