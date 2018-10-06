#ifndef MODEL_MESSAGE_H
#define MODEL_MESSAGE_H

/// APEX
#include <csapex/serialization/serialization_buffer.h>

/// PCL
#include <pcl/ModelCoefficients.h>

// ignore deprecation warning for unused code of pcl
#pragma push_macro("PCL_DEPRECATED")
#undef PCL_DEPRECATED
#define PCL_DEPRECATED(...)
#include <pcl/sample_consensus/model_types.h>
#pragma pop_macro("PCL_DEPRECATED")

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
class ModelMessage
{
public:
    ModelMessage();
    pcl::SacModel model_type;
    pcl::ModelCoefficients::Ptr coefficients;
    std::string frame_id;
    double probability;
};

SerializationBuffer& operator<<(SerializationBuffer& data, const ModelMessage& rhs);
const SerializationBuffer& operator>>(const SerializationBuffer& data, ModelMessage& rhs);

}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct convert<csapex::ModelMessage>
{
    static Node encode(const csapex::ModelMessage& rhs);
    static bool decode(const Node& node, csapex::ModelMessage& rhs);
};
}  // namespace YAML

#endif  // MODEL_MESSAGE_H
