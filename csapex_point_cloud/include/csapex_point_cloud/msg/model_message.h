#ifndef MODEL_MESSAGE_H
#define MODEL_MESSAGE_H

/// PCL
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{

class ModelMessage
{
public:
    ModelMessage();
    pcl::SacModel               model_type;
    pcl::ModelCoefficients::Ptr coefficients;
    std::string                 frame_id;
    double probability;
};

}


/// YAML
namespace YAML {
template<>
struct convert<csapex::ModelMessage> {
  static Node encode(const csapex::ModelMessage& rhs);
  static bool decode(const Node& node, csapex::ModelMessage& rhs);
};
}

#endif // MODEL_MESSAGE_H
