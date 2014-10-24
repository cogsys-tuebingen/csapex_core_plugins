#ifndef YAML_IO_HPP
#define YAML_IO_HPP

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

/// YAML
namespace YAML {

// TODO: implement
template<>
struct convert<visualization_msgs::MarkerArray> {
    static Node encode(const visualization_msgs::MarkerArray& rhs) {
        Node node;
        return node;
    }

    static bool decode(const Node& node, visualization_msgs::MarkerArray& rhs) {
        return true;
    }
};

// TODO: implement
template<>
struct convert<nav_msgs::OccupancyGrid> {
    static Node encode(const nav_msgs::OccupancyGrid& rhs) {
        Node node;
        return node;
    }

    static bool decode(const Node& node, nav_msgs::OccupancyGrid& rhs) {
        return true;
    }
};

}

#endif // YAML_IO_HPP
