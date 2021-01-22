#include "rt_runner_node.hpp"
#include "rt_runner.hpp"
#include <iostream>
#include <memory>

#include <rtt/os/main.h>

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rtcf/StartOrocosComponent.h"

RTRunnerNode::RTRunnerNode(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle) {
    rt_runner_ = std::make_shared<RTRunner>();
};

RTRunnerNode::~RTRunnerNode() { shutdown(); };

void RTRunnerNode::configure() {
    setupROS();
    loadROSParameters();
    rt_runner_->configure();
};
void RTRunnerNode::shutdown() {
    rt_runner_->shutdown();
    shutdownROS();
};

int RTRunnerNode::loop() {
    ros::spin();
    shutdown();
    ros::spinOnce();

    return 0;
};
    
void RTRunnerNode::setupROS() {
};

void RTRunnerNode::shutdownROS() {
};

void RTRunnerNode::loadROSParameters() {};
    
    void loadOrocosComponentCallback();
    void unloadOrocosComponentCallback();
    void activateRTLoopCallback();
    void deactivateRTLoopCallback();


int ORO_main(int argc, char** argv) {
    ros::init(argc, argv, "RTRunner");
    ros::NodeHandle nh("~");

    RTRunnerNode node = RTRunnerNode(nh);
    node.configure();

    return node.loop();
}


/*
 *using namespace RTT;
 *int ORO_main(int argc, char** argv) {
 *    //
 *    // Ros stuff
 *    //
 *
 *    ros::init(argc, argv, "talker");
 *
 *    ros::NodeHandle n;
 *
 *    //
 *    // Ros stuff - end
 *    //
 *
 *    //
 *    // Load dynamic library stuff
 *    //
 *
 *    std::cout << "Opening hello.so...\n";
 *    void* handle = dlopen(
 *        "/home/stefan/Dokumente/Dropbox/KIT/Informatik/Antropromatik_Praktikum/"
 *        "code/devel/lib/orocos/gnulinux/minimum_test_1/"
 *        "libminimum_test_1-gnulinux.so",
 *        RTLD_LAZY);
 *    if (!handle) {
 *        std::cerr << "Cannot open library: " << dlerror() << '\n';
 *        return 1;
 *    }
 *
 *    typedef TaskContext* create_t(std::string);
 *
 *    // load the symbol
 *    std::cout << "Loading symbol hello...\n";
 *    // reset errors
 *    dlerror();
 *
 *    create_t* creat_task = (create_t*)dlsym(handle, "createComponent");
 *
 *    const char* dlsym_error = dlerror();
 *    if (dlsym_error) {
 *        std::cerr << "Cannot load symbol 'hello': " << dlsym_error << '\n';
 *        dlclose(handle);
 *        return 1;
 *    }
 *
 *    TaskContext* task = creat_task("abc");
 *
 *    //
 *    // Load dynamic library stuff  - end
 *    //
 *
 *    Logger::In in("main()");
 *
 *    // Set log level more verbose than default,
 *    // such that we can see output :
 *    if (log().getLogLevel() < Logger::Info) {
 *        log().setLogLevel(Logger::Info);
 *        log(Info) << argv[0]
 *                  << " manually raises LogLevel to 'Info' (5). See also "
 *                     "file 'orocos.log'."
 *                  << endlog();
 *    }
 *
 *    RTT::Activity* master_one = new RTT::Activity(ORO_SCHED_RT, 99, 0.01);
 *    // a 'slave', takes over properties (period,...) of 'master_one':
 *    RTT::extras::SlaveActivity* slave_one =
 *        new RTT::extras::SlaveActivity(master_one);
 *    RTT::extras::SlaveActivity* slave_two =
 *        new RTT::extras::SlaveActivity(master_one);
 *
 *    master master("master", slave_one, slave_two);
 *
 *    // Create the activity which runs the task's engine:
 *    // master.setActivity(master_one);
 *    task->setActivity(master_one);
 *
 *    // master.configure();
 *    task->configure();
 *
 *    // master.start();
 *    task->start();
 *
 *    // log(Info) << "**** Starting the TaskBrowser       ****" << endlog();
 *    // Switch to user-interactive mode.
 *    // OCL::TaskBrowser browser(task);
 *
 *    // Accept user commands from console.
 *    // browser.loop();
 *
 *    ros::spin();
 *
 *    task->stop();
 *
 *    return 0;
 *}
 */
