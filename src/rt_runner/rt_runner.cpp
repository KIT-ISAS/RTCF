#include "rt_runner.hpp"
#include <dlfcn.h>
#include <iostream>
#include "rtt/extras/SlaveActivity.hpp"

#include <ocl/OCL.hpp>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>

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
                                   std::string componentName){
    RTT::TaskContext* task;
    bool error = createFromLibrary(componentType, componentName, task);

    RTT::extras::SlaveActivity* slave_activity =
        new RTT::extras::SlaveActivity(main_activity_);

    OrocosContainer orocos_container(task, slave_activity);

    orocosContainer_.push_back(orocos_container);

    return error;
};

void RTRunner::setSlavesOnMainContext() {
    main_context_.stop();

    /* TODO: this has to be replaced <23-01-21, Stefan Geyer> */

    std::vector<RTT::extras::SlaveActivity*> slaves;

    for (auto orocos_container : orocosContainer_) {
        slaves.push_back(orocos_container.activity);
    }

    main_context_.setSlaves(slaves);
    if (isActive) {
        main_context_.start();
    }
};

bool RTRunner::createFromLibrary(std::string componentType,
                                 std::string componentName,
                                 RTT::TaskContext*& task) {
    /* TODO: replace with function from orocos deployer <22-01-21, Stefan Geyer>
     */
    std::cout << "Opening " + componentName + "...\n";
    void* handle = dlopen(componentName.c_str(), RTLD_LAZY);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return false;
    }

    typedef RTT::TaskContext* create_t(std::string);

    // load the symbol
    std::cout << "Loading symbol hello...\n";
    // reset errors
    dlerror();

    create_t* creat_task = (create_t*)dlsym(handle, "createComponent");

    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol 'hello': " << dlsym_error << '\n';
        dlclose(handle);
        return false;
    }

    return creat_task(componentName);
};

bool RTRunner::unloadOrocosComponent() {
    main_context_.stop();
    main_context_.clearSlaves();

    if (isActive) {
        main_context_.start();
    }

    return true;
};

void RTRunner::setFrequency(float frequency) { period_ = 1 / frequency; };
void RTRunner::setPeriod(float period) { period_ = period; };
