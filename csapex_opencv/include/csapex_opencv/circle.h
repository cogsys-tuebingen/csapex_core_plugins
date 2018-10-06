#ifndef CIRCLE_H
#define CIRCLE_H
/// COMPONENT
#include <csapex_opencv/csapex_opencv_export.h>
#include <csapex/serialization/serialization_buffer.h>
#include <opencv2/core/core.hpp>
namespace csapex
{
struct Circle
{
  Circle();
  Circle(double cx, double cy, double r);
  double center_x;
  double center_y;
  double radius;
  int id;
};

SerializationBuffer& operator << (SerializationBuffer& data, const Circle& c);
const SerializationBuffer& operator >> (const SerializationBuffer& data,Circle& c);

}
#endif // CIRCLE_H
