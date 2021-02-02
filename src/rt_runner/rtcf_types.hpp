#ifndef RTCF_TYPES_H
#define RTCF_TYPES_H

#include <vector>
#include <string>

#include "ros/ros.h"

#include <ocl/OCL.hpp>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include "rtt/InputPort.hpp"
#include "rtt/base/InputPortInterface.hpp"
#include "rtt/extras/SlaveActivity.hpp"

struct mapping {
    std::string from_topic;
    std::string to_topic;
};

struct OrocosContainer {
    OrocosContainer(std::string componentType, std::string componentName,
                    bool is_start, std::vector<mapping> mappings,
                    RTT::TaskContext* taskContext,
                    RTT::extras::SlaveActivity* activity)
        : componentType_(componentType),
          componentName_(componentName),
          is_start_(is_start),
          mappings_(mappings),
          taskContext_(taskContext),
          activity_(activity) {
        getPorts();
    }

    std::string componentType_;
    std::string componentName_;
    bool is_start_;
    std::vector<mapping> mappings_;

    RTT::TaskContext* taskContext_;
    RTT::extras::SlaveActivity* activity_;

    RTT::DataFlowInterface::Ports input_ports_;
    RTT::DataFlowInterface::Ports output_ports_;

    void getPorts() {
        for (RTT::base::PortInterface* port :
             taskContext_->ports()->getPorts()) {

            ROS_DEBUG_STREAM("port of component found: " << port->getName());

            if (dynamic_cast<RTT::base::InputPortInterface*>(port)) {
                input_ports_.push_back(port);
                ROS_DEBUG_STREAM(" Port is input" << std::endl);

            } else if (dynamic_cast<RTT::base::OutputPortInterface*>(port)) {
                output_ports_.push_back(port);
                ROS_DEBUG_STREAM(" Port is output" << std::endl);

            } else {
                ROS_DEBUG_STREAM(" Port is neighter Input or Output Port"
                                 << std::endl);
            }
        }
    }
};

#endif /* RTCF_TYPES_H */
