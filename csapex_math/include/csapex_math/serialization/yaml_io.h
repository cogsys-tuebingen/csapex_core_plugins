#ifndef YAML_IO_H
#define YAML_IO_H

/// COMPONENT
#include <csapex_math/model/vector.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

/// YAML
///
///
namespace YAML
{
template <>
struct CSAPEX_MATH_EXPORT convert<csapex::math::linear::Vector>
{
    static Node encode(const csapex::math::linear::Vector& rhs);
    static bool decode(const Node& node, csapex::math::linear::Vector& rhs);
};
}  // namespace YAML

#endif  // YAML_IO_H
