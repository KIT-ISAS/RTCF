#ifndef RTCF_TYPES_H
#define RTCF_TYPES_H

#include <boost/functional/forward_adapter.hpp>
#include <memory>
#include <vector>
#include <string>

#include "ros/node_handle.h"
#include "ros/ros.h"

#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include "rtt/DataFlowInterface.hpp"
#include "rtt/InputPort.hpp"
#include "rtt/base/InputPortInterface.hpp"
#include "rtt/base/PortInterface.hpp"
#include "rtt/extras/SlaveActivity.hpp"

#include "rtcf/rtcf_extension.hpp"

struct mapping {
    std::string from_topic;
    std::string to_topic;
};

struct PortContainer {
    PortContainer(RTT::base::PortInterface* port) : port_(port) {
        getName();
        setDefaultMapping();
    }
    RTT::base::PortInterface* port_;
    std::string original_name_;
    std::string mapping_name_;

    void getName() { original_name_ = port_->getName(); }
    void setDefaultMapping() { mapping_name_ = original_name_; }
};

struct OrocosContainer {
    OrocosContainer(std::string componentType, std::string componentName,
                    std::string ns, bool is_start, bool is_sync,
                    std::vector<mapping> mappings,
                    RTT::TaskContext* taskContext,
                    RTT::extras::SlaveActivity* activity)
        : componentType_(componentType),
          componentName_(componentName),
          ns_(ns),
          is_start_(is_start),
          is_sync_(is_sync),
          mappings_(mappings),
          taskContext_(taskContext),
          activity_(activity) {
        handleNodeHandle();
        handlePorts();
        handleMappings();
    }

    std::string componentType_;
    std::string componentName_;
    std::string ns_;
    bool is_start_;
    bool is_sync_;
    std::vector<mapping> mappings_;

    RTT::TaskContext* taskContext_;
    RTT::extras::SlaveActivity* activity_;

    std::vector<PortContainer> input_ports_;
    std::vector<PortContainer> output_ports_;

    std::shared_ptr<ros::NodeHandle> node_handle_;

    void handleNodeHandle() {
        node_handle_ = std::make_shared<ros::NodeHandle>(ns_);

        if (auto taskHandle = dynamic_cast<RtcfExtension *>(taskContext_)) {
            taskHandle->node_handle_ptr_ = node_handle_.get();
        }
    };

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
            for (auto& p : input_ports_) {
                if (m.from_topic == p.original_name_) {
                    const auto resolved_mapping =
                        node_handle_->resolveName(m.to_topic);
                    p.mapping_name_ = resolved_mapping;
                    ROS_INFO_STREAM("connection input port: "
                                    << m.from_topic
                                    << " with mapping: " << m.to_topic);
                }
            }
            for (auto& p : output_ports_) {
                if (m.from_topic == p.original_name_) {
                    const auto resolved_mapping =
                        node_handle_->resolveName(m.to_topic);
                    p.mapping_name_ = resolved_mapping;
                    ROS_INFO_STREAM("connection output port: "
                                    << m.from_topic
                                    << " with mapping: " << m.to_topic);
                }
            }
        }
    }
};

/* TODO: This whole block beneth should be done better <03-02-21, Stefan Geyer> */

struct GraphOrocosContainer;

struct GraphPortContainer : PortContainer {
    GraphPortContainer(const PortContainer port_container)
        : PortContainer(port_container) {}

    bool is_connected = false;
    bool is_satisfied = false;
};

struct GraphPortMatch {
    GraphPortMatch(GraphOrocosContainer* corr_orocos_ptr,
                   GraphPortContainer* corr_port_ptr)
        : corr_orocos_ptr_(corr_orocos_ptr), corr_port_ptr_(corr_port_ptr){};
    GraphPortMatch(){};

    GraphOrocosContainer* corr_orocos_ptr_ = nullptr;
    GraphPortContainer* corr_port_ptr_ = nullptr;
};
typedef std::vector<GraphPortMatch> GraphPortMatches;

struct GraphInportContainer : GraphPortContainer {
    GraphInportContainer(const PortContainer port_container)
        : GraphPortContainer(port_container) {}

    GraphPortMatch outport_match;
};

struct GraphOutportContainer : GraphPortContainer {
    GraphOutportContainer(const PortContainer port_container)
        : GraphPortContainer(port_container) {}

    GraphPortMatches inport_matches;
};

typedef std::vector<GraphOrocosContainer> GraphOrocosContainers;

struct GraphOrocosContainer : OrocosContainer {
    GraphOrocosContainer(const OrocosContainer orocos_container)
        : OrocosContainer(orocos_container) {
        for (const auto& p : orocos_container.input_ports_) {
            input_ports_.push_back(GraphInportContainer(p));
        }
        for (const auto& p : orocos_container.output_ports_) {
            output_ports_.push_back(GraphOutportContainer(p));
        }
    }

    bool is_queued = false;

    std::vector<GraphInportContainer> input_ports_;
    std::vector<GraphOutportContainer> output_ports_;

    std::vector<GraphOrocosContainer*> connected_container_;

    bool is_satisfied() {
        bool is_satisfied = true;

        for (const GraphInportContainer& p : input_ports_) {
          if (p.is_connected && !p.is_satisfied) {
            is_satisfied = false;
          }
        }
        return is_satisfied;
    }

    std::vector<GraphOrocosContainer*> enqueue_and_satisfy_nodes() {
        std::vector<GraphOrocosContainer*> to_enqueue;

        for (auto& output_port : output_ports_) {
            for (auto& inport_match : output_port.inport_matches) {
                inport_match.corr_port_ptr_->is_satisfied = true;
                if (!inport_match.corr_orocos_ptr_->is_queued) {
                    inport_match.corr_orocos_ptr_->is_queued = true;
                    to_enqueue.push_back(inport_match.corr_orocos_ptr_);
                }
            }
        }

        return to_enqueue;
    }
};

#endif /* RTCF_TYPES_H */
