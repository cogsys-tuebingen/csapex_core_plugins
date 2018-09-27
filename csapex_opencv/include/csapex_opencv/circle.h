#ifndef CIRCLE_H
#define CIRCLE_H
/// COMPONENT
#include <csapex_opencv/csapex_opencv_export.h>
#include <csapex/serialization/serialization_buffer.h>
/// SYSTEM
#include <opencv2/core/core.hpp>

namespace csapex
{
struct CSAPEX_OPENCV_EXPORT Circle
{
  double center_x;
  double center_y;
  double radius;
};

SerializationBuffer& operator << (SerializationBuffer& data, const Circle& c);
const SerializationBuffer& operator >> (const SerializationBuffer& data,Circle& c);

namespace connection_types {
struct CSAPEX_OPENCV_EXPORT CircleMessage : public MessageTemplate<Circle, CircleMessage>
{
  typedef std::shared_ptr<CircleMessage> Ptr;
  typedef std::shared_ptr<CircleMessage const> ConstPtr;

  CircleMessage() {}
};


/// TRAITS
template <>
struct type<CircleMessage> {
    static std::string name() {
        return "CircleMessage";
    }
};
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::CircleMessage> {
    static Node encode(const csapex::connection_types::CircleMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::CircleMessage& rhs);
};
template<>
struct convert<csapex::connection_types::CircleMessage> {
    static Node encode(const csapex::connection_types::CircleMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::CircleMessage& rhs);
};
}
#endif // CIRCLE_H
