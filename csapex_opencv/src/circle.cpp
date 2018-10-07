
/// HEADER
#include <csapex_opencv/circle.h>

/// COMPONENT
//#include <yaml-cpp/yaml.h>

using namespace csapex;

Circle::Circle() : center_x(0), center_y(0), radius(0), id(-1)
{
}

Circle::Circle(double cx, double cy, double r) : center_x(cx), center_y(cy), radius(r), id(-1)
{
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const Circle& c)
{
    data << c.center_x;
    data << c.center_y;
    data << c.radius;
    data << c.id;
    return data;
}

const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, Circle& c)
{
    data >> c.center_x;
    data >> c.center_y;
    data >> c.radius;
    data >> c.id;
    return data;
}

YAML::Node YAML::convert<csapex::Circle>::encode(const csapex::Circle& rhs)
{
    YAML::Node node;
    node["center_x"] = rhs.center_x;
    node["center_y"] = rhs.center_y;
    node["radius"] = rhs.radius;
    node["id"] = rhs.radius;
    return node;
}

bool YAML::convert<csapex::Circle>::decode(const YAML::Node& node, csapex::Circle& rhs)
{
    rhs.center_x = node["center_x"].as<double>();
    rhs.center_y = node["center_y"].as<double>();
    rhs.radius = node["radius"].as<double>();
    rhs.id = node["id"].as<int>();
    return true;
}
