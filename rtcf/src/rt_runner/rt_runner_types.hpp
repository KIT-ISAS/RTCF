#ifndef RT_RUNNER_TYPES_H
#define RT_RUNNER_TYPES_H

#include <boost/functional/forward_adapter.hpp>
#include <memory>
#include <regex>
#include <rtt/Activity.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include <string>
#include <vector>

#include "ros/node_handle.h"
#include "ros/ros.h"
#include "rtcf/rtcf_extension.hpp"
#include "rtcf/rtcf_types.hpp"
#include "rtt/DataFlowInterface.hpp"
#include "rtt/InputPort.hpp"
#include "rtt/base/InputPortInterface.hpp"
#include "rtt/base/PortInterface.hpp"
#include "rtt/extras/SlaveActivity.hpp"

struct PortContainer {
    PortContainer(RTT::base::PortInterface* port) : port(port) {
        getName();
        setDefaultMapping();
    }
    RTT::base::PortInterface* port;
    std::string original_name;
    std::string mapping_name;

    void getName() { original_name = port->getName(); }
    void setDefaultMapping() { mapping_name = original_name; }
};

struct ComponentContainer {
    ComponentContainer(const LoadAttributes& attributes, RTT::TaskContext* task_context,
                       RTT::extras::SlaveActivity* activity)
        : attributes(attributes), task_context(task_context), activity(activity) {
        handleNodeHandle();
        handlePorts();
        handleMappings();
    }

    // externally provided information
    LoadAttributes attributes;
    RTT::TaskContext* task_context;
    RTT::extras::SlaveActivity* activity;

    // internally constructed information
    std::vector<PortContainer> input_ports;
    std::vector<PortContainer> output_ports;
    std::shared_ptr<ros::NodeHandle> node_handle;

    void handleNodeHandle() {
        // create a node handle in the components namespace
        // TODO: include remappings, don't use pointers
        node_handle_ = std::make_shared<ros::NodeHandle>(attributes.ns);

        if (auto task_handle = dynamic_cast<RtcfExtension*>(task_context)) {
            task_handle->node_handle_ptr_ = node_handle_.get();
        }
    };

    void handlePorts() {
        for (RTT::base::PortInterface* port : task_context->ports()->getPorts()) {
            PortContainer port_container(port);
            std::stringstream ss;
            ss << "Port of component found: " << port_container.original_name_ << " ";

            if (dynamic_cast<RTT::base::InputPortInterface*>(port)) {
                input_ports_.push_back(port_container);
                ss << "(input)";

            } else if (dynamic_cast<RTT::base::OutputPortInterface*>(port)) {
                output_ports_.push_back(port_container);
                ss << "(outpout)";

            } else {
                ss << "(neither input nor output)";
            }
            ROS_INFO_STREAM(ss.str());
        }
    }

    void handleMappings() {
        for (const auto& m : mappings_) {
            for (auto& p : input_ports_) {
                if (m.from_topic == p.original_name_) {
                    const auto resolved_mapping = node_handle_->resolveName(m.to_topic);
                    p.mapping_name_             = resolved_mapping;
                    ROS_INFO_STREAM("connection input port: " << m.from_topic << " with mapping: " << m.to_topic);
                }
            }
            for (auto& p : output_ports_) {
                if (m.from_topic == p.original_name_) {
                    const auto resolved_mapping = node_handle_->resolveName(m.to_topic);
                    p.mapping_name_             = resolved_mapping;
                    ROS_INFO_STREAM("connection output port: " << m.from_topic << " with mapping: " << m.to_topic);
                }
            }
        }
    }
};

/* TODO: This whole block beneth should be done better <03-02-21, Stefan Geyer> */

struct GraphOrocosContainer;

struct GraphPortContainer : PortContainer {
    GraphPortContainer(const PortContainer port_container) : PortContainer(port_container) {}

    bool is_connected = false;
    bool is_satisfied = false;
};

struct GraphPortMatch {
    GraphPortMatch(GraphOrocosContainer* corr_orocos_ptr, GraphPortContainer* corr_port_ptr)
        : corr_orocos_ptr_(corr_orocos_ptr), corr_port_ptr_(corr_port_ptr){};
    GraphPortMatch(){};

    GraphOrocosContainer* corr_orocos_ptr_ = nullptr;
    GraphPortContainer* corr_port_ptr_     = nullptr;
};
typedef std::vector<GraphPortMatch> GraphPortMatches;

struct GraphInportContainer : GraphPortContainer {
    GraphInportContainer(const PortContainer port_container) : GraphPortContainer(port_container) {}

    GraphPortMatch outport_match;
};

struct GraphOutportContainer : GraphPortContainer {
    GraphOutportContainer(const PortContainer port_container) : GraphPortContainer(port_container) {}

    GraphPortMatches inport_matches;
};

typedef std::vector<GraphOrocosContainer> GraphOrocosContainers;

struct GraphOrocosContainer : ComponentContainer {
    GraphOrocosContainer(const ComponentContainer orocos_container) : ComponentContainer(orocos_container) {
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

    bool topic_is_ignored(const std::string topic) {
        auto topic_is_ignored_regex = std::regex(topics_ignore_for_graph_);
        return std::regex_match(topic, topic_is_ignored_regex);
    }

    bool is_satisfied() {
        bool is_satisfied = true;

        for (const GraphInportContainer& p : input_ports_) {
            if ((p.is_connected && !p.is_satisfied) && !topic_is_ignored(p.mapping_name_)) {
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

#endif /* RT_RUNNER_TYPES_H */
