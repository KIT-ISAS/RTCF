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

#include <dlfcn.h>

#include <rtt/OperationCaller.hpp>

#include <ocl/OCL.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"

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


//
// Ros stuff
//

ros::init(argc, argv, "talker");

ros::NodeHandle n;

//
// Ros stuff - end
//




//
// Load dynamic library stuff
//

    std::cout << "Opening hello.so...\n";
    void* handle = dlopen("/home/stefan/Dokumente/Dropbox/KIT/Informatik/Antropromatik_Praktikum/code/devel/lib/orocos/gnulinux/minimum_test_1/libminimum_test_1-gnulinux.so", RTLD_LAZY);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return 1;
    }

    typedef TaskContext* create_t(std::string);

     // load the symbol    
    std::cout << "Loading symbol hello...\n";
    // reset errors    
    dlerror();

    create_t* creat_task = (create_t*) dlsym(handle, "createComponent");

    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol 'hello': " << dlsym_error << '\n';
        dlclose(handle);
        return 1;
    }

    TaskContext* task = creat_task("abc");

//
// Load dynamic library stuff  - end
//


    Logger::In in("main()");

    // Set log level more verbose than default,
    // such that we can see output :
    if (log().getLogLevel() < Logger::Info) {
        log().setLogLevel(Logger::Info);
        log(Info) << argv[0]
                  << " manually raises LogLevel to 'Info' (5). See also "
                     "file 'orocos.log'."
                  << endlog();
    }

    RTT::Activity* master_one = new RTT::Activity(ORO_SCHED_RT, 99, 0.01);
    // a 'slave', takes over properties (period,...) of 'master_one':
    RTT::extras::SlaveActivity* slave_one =
        new RTT::extras::SlaveActivity(master_one);
    RTT::extras::SlaveActivity* slave_two =
        new RTT::extras::SlaveActivity(master_one);

    master master("master", slave_one, slave_two);

    // Create the activity which runs the task's engine:
    //master.setActivity(master_one);
    task->setActivity(master_one);

    //master.configure();
    task->configure();

    //master.start();
    task->start();

    //log(Info) << "**** Starting the TaskBrowser       ****" << endlog();
    // Switch to user-interactive mode.
    //OCL::TaskBrowser browser(task);

    // Accept user commands from console.
    //browser.loop();

    ros::spin();

    task->stop();

    return 0;
}
