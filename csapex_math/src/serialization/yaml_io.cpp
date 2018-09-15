/// HEADER
#include <csapex_math/serialization/yaml_io.h>


/// YAML
namespace YAML {
CSAPEX_MATH_EXPORT Node convert<csapex::math::linear::Vector>::encode(const csapex::math::linear::Vector& rhs)
{
    Node node;
    node["rows"] = rhs.rows();
    node["data"] = rhs.getDataRef();

    return node;
}

CSAPEX_MATH_EXPORT bool convert<csapex::math::linear::Vector>::decode(const Node& node, csapex::math::linear::Vector& rhs)
{
    std::vector<double> v = node["data"].as<std::vector<double>>(v);
    rhs = csapex::math::linear::Vector(v);
    return true;
}
}
