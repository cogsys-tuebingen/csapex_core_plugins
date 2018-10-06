/// HEADER
#include <csapex_math/serialization/yaml_io.h>

/// YAML
namespace YAML
{
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

CSAPEX_MATH_EXPORT Node convert<csapex::math::linear::Matrix>::encode(const csapex::math::linear::Matrix& rhs)
{
    Node node;
    node["rows"] = rhs.rows();
    node["cols"] = rhs.cols();
    node["data"] = rhs.getDataRef();

    return node;
}

CSAPEX_MATH_EXPORT bool convert<csapex::math::linear::Matrix>::decode(const Node& node, csapex::math::linear::Matrix& rhs)
{
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    std::vector<double> v = node["data"].as<std::vector<double>>(v);
    rhs = csapex::math::linear::Matrix(rows, cols, v);
    return true;
}
}  // namespace YAML
