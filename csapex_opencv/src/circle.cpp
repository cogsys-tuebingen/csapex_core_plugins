
/// HEADER
#include <csapex_opencv/circle.h>

/// COMPONENT
#include <yaml-cpp/yaml.h>

using namespace csapex;

Circle::Circle() : center_x(0), center_y(0), radius(0)
{
}

Circle::Circle(double cx, double cy, double r) : center_x(cx), center_y(cy), radius(r)
{
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const Circle& c)
{
    data << c.center_x;
    data << c.center_y;
    data << c.radius;
    return data;
}

const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, Circle& c)
{
    data >> c.center_x;
    data >> c.center_y;
    data >> c.radius;
    return data;
}
