#ifndef LINEAR_MATRIX_H
#define LINEAR_MATRIX_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// COMPONENT
#include <csapex_math/csapex_math_export.h>
#include <csapex_math/model/matrix.h>
#include <csapex_math/serialization/binary_io.h>

namespace csapex
{
namespace connection_types
{
class LinearMatrixMessage : public MessageTemplate<math::linear::Matrix, LinearMatrixMessage>
{
    using Parent = MessageTemplate<math::linear::Matrix, LinearMatrixMessage>;

public:
    LinearMatrixMessage() = default;
    LinearMatrixMessage(math::linear::Matrix v, const std::string& frame_id = "/", Message::Stamp stamp = 0);
};

/// TRAITS
template <>
struct CSAPEX_MATH_EXPORT type<LinearMatrixMessage>
{
    static std::string name()
    {
        return "linear::Matrix";
    }
};

}  // namespace connection_types

}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_MATH_EXPORT convert<csapex::connection_types::LinearMatrixMessage>
{
    static Node encode(const csapex::connection_types::LinearMatrixMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::LinearMatrixMessage& rhs);
};
}  // namespace YAML

#endif  // LINEAR_MATRIX_H
