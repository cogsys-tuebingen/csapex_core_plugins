/// HEADER
#include <csapex_point_cloud/msg/model_message.h>

using namespace csapex;

ModelMessage::ModelMessage()
{

}

/// YAML
namespace YAML {

Node convert<csapex::ModelMessage>::encode(const csapex::ModelMessage& rhs) {
    Node node;

    node["model_type"] = static_cast<int> (rhs.model_type);
    node["frame_id"] = rhs.frame_id;
    node["probability"] = rhs.probability;

    node["coefficients/header/frame_id"] = rhs.coefficients->header.frame_id;
    node["coefficients/header/seq"] = rhs.coefficients->header.seq;
    node["coefficients/header/stamp"] = rhs.coefficients->header.stamp;

    node["coefficients/values"] = rhs.coefficients->values;

    return node;
}

bool convert<csapex::ModelMessage>::decode(const Node& node, csapex::ModelMessage& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    rhs.model_type = static_cast<pcl::SacModel> (node["model_type"].as<int>());
    rhs.frame_id = node["frame_id"].as<std::string>();
    rhs.probability = node["probability"].as<double>();

    rhs.coefficients.reset(new pcl::ModelCoefficients);
    rhs.coefficients->header.frame_id = node["coefficients/header/frame_id"].as<std::string>();
    rhs.coefficients->header.seq = node["coefficients/header/seq"].as<unsigned int>();
    rhs.coefficients->header.stamp = node["coefficients/header/stamp"].as<unsigned long>();

    rhs.coefficients->values = node["coefficients/values"].as<std::vector<float> >();
    return true;
}
}
