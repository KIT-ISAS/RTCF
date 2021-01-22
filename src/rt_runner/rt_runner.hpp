#ifndef RT_RUNNER_H
#define RT_RUNNER_H

#include "main_context.hpp"

#include "rtt/extras/SlaveActivity.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include <vector>


struct OrocosContainer {
    RTT::TaskContext* taskContext;
    RTT::extras::SlaveActivity* activity;
};


class RTRunner {
   private:
    bool infererOrder();

    bool createFromLibrary(std::string componentType, std::string componentName,
                           RTT::TaskContext*& task);

    void setSlavesOnMainContext();


    bool isActive;

   public:
    RTRunner();

    void configure();
    void shutdown();

    bool loadOrocosComponent(std::string componentType,
                             std::string componentName);
    bool unloadOrocosComponent();

    void activateRTLoopCallback();
    void deactivateRTLoopCallback();

    void setFrequency(float frequency);
    void setPeriod(float period);

    RTT::Activity* main_activity_;
    std::vector<OrocosContainer> orocosContainer_;

    float period_;
    MainContext main_context_;
};


#endif /* RT_RUNNER_H */
