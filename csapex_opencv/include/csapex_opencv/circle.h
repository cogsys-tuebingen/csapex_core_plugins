#ifndef CIRCLE_H
#define CIRCLE_H
/// COMPONENT
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/utility/yaml.h>
#include <csapex_opencv/csapex_opencv_export.h>

#include <opencv2/core/core.hpp>
namespace csapex
{
struct Circle
{
    Circle();
    Circle(double cx, double cy, double r);
    inline const double& x() const {return center_x;}
    inline const double& y() const{return center_y;}
    inline double& x(){return center_x;}
    inline double& y(){return center_y;}
    double center_x;
    double center_y;
    double radius;
    int id;
};

SerializationBuffer& operator<<(SerializationBuffer& data, const Circle& c);
const SerializationBuffer& operator>>(const SerializationBuffer& data, Circle& c);

}  // namespace csapex

namespace YAML {
template<>
struct convert<csapex::Circle> {
  static Node encode(const csapex::Circle& rhs);
  static bool decode(const Node& node, csapex::Circle& rhs);
};

}  // namespace YAML
#endif  // CIRCLE_H
