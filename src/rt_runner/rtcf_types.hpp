#ifndef RTCF_TYPES_H
#define RTCF_TYPES_H

#include <vector>
#include <string>

#include "ros/ros.h"

#include <ocl/OCL.hpp>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include "rtt/DataFlowInterface.hpp"
#include "rtt/InputPort.hpp"
#include "rtt/base/InputPortInterface.hpp"
#include "rtt/base/PortInterface.hpp"
#include "rtt/extras/SlaveActivity.hpp"

struct mapping {
    std::string from_topic;
    std::string to_topic;
};

struct PortContainer {
    PortContainer(RTT::base::PortInterface* port) : port_(port) { getName(); }
    RTT::base::PortInterface* port_;
    std::string original_name_;
    std::string mapping_name_;

    void getName() { original_name_ = port_->getName(); }
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
        handlePorts();
        handleMappings();
    }

    std::string componentType_;
    std::string componentName_;
    bool is_start_;
    std::vector<mapping> mappings_;

    RTT::TaskContext* taskContext_;
    RTT::extras::SlaveActivity* activity_;

    std::vector<PortContainer> input_ports_;
    std::vector<PortContainer> output_ports_;

    void handlePorts() {
      for (RTT::base::PortInterface *port : taskContext_->ports()->getPorts()) {

        PortContainer portContainer(port);
        ROS_DEBUG_STREAM(
            "port of component found: " << portContainer.original_name_);

        if (dynamic_cast<RTT::base::InputPortInterface *>(port)) {
          input_ports_.push_back(portContainer);
          ROS_DEBUG_STREAM(" Port is input" << std::endl);

        } else if (dynamic_cast<RTT::base::OutputPortInterface *>(port)) {
          output_ports_.push_back(portContainer);
          ROS_DEBUG_STREAM(" Port is output" << std::endl);

        } else {
          ROS_DEBUG_STREAM(" Port is neighter Input or Output Port"
                           << std::endl);
        }
      }
    }

    void handleMappings() {
        for (const auto& m : mappings_) {
            std::cout <<"got called" << std::endl;
            for (auto& p : input_ports_) {
                if (m.from_topic == p.original_name_) {
                    p.mapping_name_ = m.to_topic;
                    ROS_INFO_STREAM("connection input port: "
                                     << m.from_topic
                                     << " with mapping: " << m.to_topic);
                }
            }
            for (auto& p : output_ports_) {
                if (m.from_topic == p.original_name_) {
                    p.mapping_name_ = m.to_topic;
                    ROS_INFO_STREAM("connection output port: "
                                     << m.from_topic
                                     << " with mapping: " << m.to_topic);
                }
            }
        }
    }
};

#endif /* RTCF_TYPES_H */
