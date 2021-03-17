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
    std::string mapped_name;

    void getName() { original_name = port->getName(); }
    void setDefaultMapping() { mapped_name = original_name; }
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
    ros::NodeHandle node_handle;

    void handleNodeHandle() {
        // create a node handle in the components namespace
        ros::M_string remaps_ros;
        for (const auto& m : attributes.mappings) {
            remaps_ros[m.from_topic] = m.to_topic;
        }
        node_handle = ros::NodeHandle(attributes.ns, remaps_ros);

        // TODO: don't use pointers and think how private parameters can be resolved
        if (auto task_handle = dynamic_cast<RtcfExtension*>(task_context)) {
            task_handle->node_handle_ptr_ = &node_handle;
        }
    };

    void handlePorts() {
        for (RTT::base::PortInterface* port : task_context->ports()->getPorts()) {
            PortContainer port_container(port);
            std::stringstream ss;
            ss << "Port of component " << attributes.name << " found: " << port_container.original_name << " ";

            if (dynamic_cast<RTT::base::InputPortInterface*>(port)) {
                input_ports.push_back(port_container);
                ss << "(input)";

            } else if (dynamic_cast<RTT::base::OutputPortInterface*>(port)) {
                output_ports.push_back(port_container);
                ss << "(outpout)";

            } else {
                ss << "(neither input nor output)";
            }
            ROS_INFO_STREAM(ss.str());
        }
    }

    void handleMappings() {
        for (auto& p : input_ports) {
            // resolveName takes into accounts remaps added to this nodehandle
            // TODO: private names are not handled correctly
            const auto resolved_name = node_handle.resolveName(p.original_name, true);
            p.mapped_name            = resolved_name;
            ROS_INFO_STREAM("Input port " << p.original_name << " mapped to " << p.mapped_name);
        }
        for (auto& p : output_ports) {
            const auto resolved_name = node_handle.resolveName(p.original_name, true);
            p.mapped_name            = resolved_name;
            ROS_INFO_STREAM("Input port " << p.original_name << " mapped to " << p.mapped_name);
        }
    }
};

using ComponentContainerVector = std::vector<ComponentContainer>;
using RTOrder                  = std::vector<const ComponentContainer*>;

// Type used to store the dependencies of a component.
// They key is the component information, the value is a set of predecessors or successors.
using ComponentPredecessorMap = std::map<const ComponentContainer*, std::set<const ComponentContainer*>>;
using ComponentSuccessorMap   = std::map<const ComponentContainer*, std::set<const ComponentContainer*>>;

// Type used for storing internal connections
// This exploits the fact, that an input is only supplied by one output in a control system
// For this reason, the key is the input port information and the value the output port information
using InternalConnectionsMap = std::map<const PortContainer*, const PortContainer*>;

using SlaveActivityVector = std::vector<RTT::extras::SlaveActivity*>;

#endif /* RT_RUNNER_TYPES_H */
