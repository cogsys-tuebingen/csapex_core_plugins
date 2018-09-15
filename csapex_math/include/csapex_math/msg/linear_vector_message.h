#ifndef LINEAR_VECTOR_H
#define LINEAR_VECTOR_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// COMPONENT
#include <csapex_math/model/vector.h>
#include <csapex_math/csapex_math_export.h>
#include <csapex_math/serialization/binary_io.h>


namespace csapex {
namespace connection_types {

class LinearVectorMessage : public MessageTemplate<math::linear::Vector, LinearVectorMessage>
{
};



/// TRAITS
template <>
struct CSAPEX_MATH_EXPORT type<LinearVectorMessage> {
    static std::string name() {
        return "linear::Vector";
    }
};

} // namespace connection_types

} // namespace csapex

/// YAML
namespace YAML {
template<>
struct CSAPEX_MATH_EXPORT convert<csapex::connection_types::LinearVectorMessage> {
  static Node encode(const csapex::connection_types::LinearVectorMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::LinearVectorMessage& rhs);
};
}

#endif // LINEAR_VECTOR_H
