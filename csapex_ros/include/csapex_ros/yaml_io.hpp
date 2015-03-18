#ifndef ROS_YAML_IO_HPP
#define ROS_YAML_IO_HPP

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

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

// TODO: implement
template<>
struct convert<geometry_msgs::Pose> {
    static Node encode(const geometry_msgs::Pose& rhs) {
        Node node;
        return node;
    }

    static bool decode(const Node& node, geometry_msgs::Pose& rhs) {
        return true;
    }
};

// TODO: implement
template<>
struct convert<geometry_msgs::PoseStamped> {
    static Node encode(const geometry_msgs::PoseStamped& rhs) {
        Node node;
        return node;
    }

    static bool decode(const Node& node, geometry_msgs::PoseStamped& rhs) {
        return true;
    }
};

// TODO: implement
template<>
struct convert<nav_msgs::Odometry> {
    static Node encode(const nav_msgs::Odometry& rhs) {
        Node node;
        return node;
    }

    static bool decode(const Node& node, nav_msgs::Odometry& rhs) {
        return true;
    }
};

}

#endif // ROS_YAML_IO_HPP
