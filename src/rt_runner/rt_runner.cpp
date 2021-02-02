#include "rt_runner.hpp"
#include <dlfcn.h>
#include <iostream>
#include "rtt/extras/SlaveActivity.hpp"

#include "ros/ros.h"

#include <ocl/OCL.hpp>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include <vector>

RTRunner::RTRunner() : main_context_("main_context"){};

void RTRunner::configure() {
    isActive = false;
    main_activity_ = new RTT::Activity(ORO_SCHED_RT, 98);
    main_context_.setActivity(main_activity_);
    main_context_.configure();
};
void RTRunner::shutdown() { deactivateRTLoop(); };

void RTRunner::activateRTLoop() {
    main_activity_->setPeriod(period_);
    main_context_.start();
    isActive = true;
};

void RTRunner::deactivateRTLoop() {
    main_context_.stop();
    isActive = false;
};

bool RTRunner::loadOrocosComponent(std::string componentType,
                                   std::string componentName, bool is_start,
                                   std::vector<mapping> mappings) {
    RTT::TaskContext* task;
    bool error = createFromLibrary(componentType, componentName, task);

    RTT::extras::SlaveActivity* slave_activity =
        new RTT::extras::SlaveActivity(main_activity_);

    task->setActivity(slave_activity);
    task->configure();

    OrocosContainer orocos_container(componentType, componentName, is_start, mappings, task, slave_activity);

    orocosContainer_.push_back(orocos_container);

    buildRTOrder();
    setSlavesOnMainContext();

    return error;
};

void RTRunner::setSlavesOnMainContext() {
    main_context_.stop();

    /* TODO: this has to be replaced <23-01-21, Stefan Geyer> */

    std::vector<RTT::extras::SlaveActivity*> slaves;

    for (auto orocos_container : orocosContainer_) {
        slaves.push_back(orocos_container.activity_);
    }

    main_context_.clearSlaves();
    main_context_.setSlaves(slaves);
    if (isActive) {
        main_context_.start();
    }
};

bool RTRunner::createFromLibrary(std::string componentType,
                                 std::string componentName,
                                 RTT::TaskContext*& task) {

    // Try all paths in LD_LIBRARY_PATH to load the orocos component
    const std::string LD_LIBRARY_PATH = getenv("LD_LIBRARY_PATH");
    std::stringstream ss(LD_LIBRARY_PATH);
    std::string item;
    std::vector<std::string> paths;
    while (std::getline(ss, item, ':')) {
        paths.push_back(item);
    }

    ROS_INFO_STREAM("Opening " + componentName + "...\n");

    void* handle;
    for (const auto base_path : paths) {
        std::string path = base_path + "/orocos/gnulinux/" + componentType + "/lib" + componentType + "-gnulinux.so";
        ROS_INFO_STREAM("trying path to load orcos component: " << path);
        handle = dlopen(path.c_str(), RTLD_NOW);
        if (handle) {
            break;
        }
    }

    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return false;
    }

    typedef RTT::TaskContext* create_t(std::string);

    // load the symbol
    ROS_INFO_STREAM("Loading symbol " + componentName +" ...\n");
    // reset errors
    dlerror();

    create_t* creat_task = (create_t*)dlsym(handle, "createComponent");

    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        ROS_ERROR_STREAM("Cannot load symbol 'hello': " << dlsym_error << '\n');
        dlclose(handle);
        return false;
    }

    task = creat_task(componentName.c_str());
    return true;
};

bool RTRunner::unloadOrocosComponent(std::string componentName) {
    /* TODO:  <01-02-21, Stefan Geyer> */
    main_context_.stop();

    buildRTOrder();
    setSlavesOnMainContext();

    if (isActive) {
        main_context_.start();
    }

    return true;
};

void RTRunner::buildRTOrder(){
    /* TODO: Fix me <01-02-21, Stefan Geyer> */
};

void RTRunner::setFrequency(float frequency) { period_ = 1 / frequency; };
void RTRunner::setPeriod(float period) { period_ = period; };
