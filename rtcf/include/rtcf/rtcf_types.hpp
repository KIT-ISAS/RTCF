#ifndef RTCF_TYPES_HPP
#define RTCF_TYPES_HPP

#include <iostream>
#include <string>
#include <vector>

struct Mapping {
    std::string from_topic;
    std::string to_topic;
};

inline std::ostream& operator<<(std::ostream& os, const Mapping& m) {
    os << " from: " << m.from_topic << ", to: " << m.to_topic << std::endl;
    return os;
}

struct LoadAttributes {
    // things that exist in a normal ROS node
    std::vector<Mapping> mappings;
    std::string name;
    std::string ns;
    // component specification (equal to ROS package and node-type)
    std::string rt_package = "";
    std::string rt_type    = "";
    // RTCF specific stuff
    std::string topics_ignore_for_graph;
    bool is_first = false;
    bool is_sync  = false;
};

inline std::ostream& operator<<(std::ostream& os, const LoadAttributes& a) {
    os << "Name: " << a.name << std::endl;
    os << "Namespace: " << a.ns << std::endl;
    os << "RT-Package: " << a.rt_package << std::endl;
    os << "RT-Type: " << a.rt_type << std::endl;
    os << "is_first: " << a.is_first << ", ";
    os << "is_sync: " << a.is_sync << std::endl;
    os << "Topics to ignore for graph:" << a.topics_ignore_for_graph << std::endl;
    os << "Mappings: " << std::endl;
    for (const auto& m : a.mappings) {
        os << m;
    };
    return os;
}

struct UnloadAttributes {
    std::string name;
    std::string ns;
};

inline std::ostream& operator<<(std::ostream& os, const UnloadAttributes& a) {
    os << "Name: " << a.name << std::endl;
    os << "Namespace: " << a.ns << std::endl;
    return os;
}

#endif  // RTCF_TYPES_HPP
