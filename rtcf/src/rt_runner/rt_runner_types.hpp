#ifndef RT_RUNNER_TYPES_H
#define RT_RUNNER_TYPES_H

#include <ros/node_handle.h>
#include <ros/ros.h>

#include <boost/functional/forward_adapter.hpp>
#include <memory>
#include <regex>
#include <rtcf/rtcf_extension.hpp>
#include <rtcf/rtcf_types.hpp>
#include <rtt/Activity.hpp>
#include <rtt/DataFlowInterface.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/base/InputPortInterface.hpp>
#include <rtt/base/PortInterface.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <string>
#include <vector>

#include "rtcf/rt_rosconsole_logging.hpp"

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
    ComponentContainer(const LoadAttributes& attributes, RTT::TaskContext* task_context) :
        attributes(attributes), task_context(task_context) {
        handleIgnoredTopicsRegex();
        handleNodeHandle();
        handleLogger();
    }

    // externally provided information
    LoadAttributes attributes;
    RTT::TaskContext* task_context;

    // internally constructed information
    std::vector<PortContainer> input_ports;
    std::vector<PortContainer> output_ports;
    ros::NodeHandlePtr node_handle;
    ros::NodeHandlePtr node_handle_private;

    // regex
    std::regex topics_ignore_regex;

    void handleIgnoredTopicsRegex() {
        try {
            topics_ignore_regex = std::regex(attributes.topics_ignore_for_graph);
        } catch (std::regex_error e) {
            ROS_ERROR_STREAM("Invalid regex in topics_ignore_for_graph for component "
                             << attributes.name << " received. Falling back to default '.*'.");
            topics_ignore_regex = std::regex(".*");
        }
    }

    void handleNodeHandle() {
        // create a node handle in the components namespace
        ros::M_string remaps_ros;
        for (const auto& m : attributes.mappings) {
            remaps_ros[m.from_topic] = m.to_topic;
        }

        // we create to node handles to be able to resolve private parameters
        // this is the same way ros::nodelet does it
        node_handle         = boost::make_shared<ros::NodeHandle>(attributes.ns, remaps_ros);
        node_handle_private = boost::make_shared<ros::NodeHandle>(attributes.name, remaps_ros);

        // forward it to the component if it can make use of it
        if (auto task_handle = dynamic_cast<RtcfExtension*>(task_context)) {
            task_handle->nh_         = node_handle;
            task_handle->nh_private_ = node_handle_private;
        }
    }

    void handleLogger() {
        // create a logger for the component when it uses the extension
        if (auto task_handle = dynamic_cast<RtcfExtension*>(task_context)) {
            // std::string last_name = attributes.name.substr(attributes.name.rfind("/")+1);
            // std::string logger_name = ROSCONSOLE_DEFAULT_NAME"." + last_name;
            std::string logger_name = ROSCONSOLE_DEFAULT_NAME "." + attributes.name;
            task_handle->logger_    = RtRosconsoleLogging::getInstance().getLoggerInstance(logger_name);
            assert(task_handle->logger_);
        }
    }

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
                ss << "(output)";

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
            const auto resolved_name = node_handle->resolveName(p.original_name, true);
            p.mapped_name            = resolved_name;
            ROS_INFO_STREAM("Input port " << p.original_name << " mapped to " << p.mapped_name);
        }
        for (auto& p : output_ports) {
            const auto resolved_name = node_handle->resolveName(p.original_name, true);
            p.mapped_name            = resolved_name;
            ROS_INFO_STREAM("Input port " << p.original_name << " mapped to " << p.mapped_name);
        }
    }

    void handlePostConfiguration() {
        handlePorts();
        handleMappings();
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

#endif /* RT_RUNNER_TYPES_H */
