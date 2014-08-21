#include <csapex_point_cloud/indeces_message.h>

using namespace csapex::connection_types;

PointIndecesMessage::PointIndecesMessage() :
    MessageTemplate<pcl::PointIndices::Ptr, PointIndecesMessage>("/")
{
    value.reset(new pcl::PointIndices);
}

void PointIndecesMessage::writeYaml(YAML::Emitter &yaml) const
{
    yaml << YAML::Flow << YAML::Key << "indeces" << YAML::Value;
    yaml << YAML::BeginSeq;
    for(std::vector<int>::const_iterator it = value->indices.begin() ;
        it != value->indices.end() ; ++it)
    {
        yaml << *it;
    }
    yaml << YAML::EndSeq;
}

void PointIndecesMessage::readYaml(const YAML::Node &node)
{
}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::PointIndecesMessage>::encode(const csapex::connection_types::PointIndecesMessage& rhs) {
    Node node;
    node["indeces"] = *rhs.value;
    return node;
}

bool convert<csapex::connection_types::PointIndecesMessage>::decode(const Node& node, csapex::connection_types::PointIndecesMessage& rhs) {
    std::cerr << "PointIndecesMessage can't be decoded yet" << std::endl;
    return true;
}


Node convert<pcl::PointIndices>::encode(const pcl::PointIndices& rhs) {
    Node node;
    node["indices"] = rhs.indices;
    node["header/frame_id"] = rhs.header.frame_id;
    node["header/seq"] = rhs.header.seq;
    node["header/stamp"] = rhs.header.stamp;

    return node;
}

bool convert<pcl::PointIndices>::decode(const Node& node, pcl::PointIndices& rhs) {
    if(!node.IsMap()) {
        return false;
    }
    rhs.indices = node["indices"].as<std::vector<int> >();
    rhs.header.frame_id = node["header/frame_id"].as<std::string>();
    rhs.header.seq = node["header/seq"].as<unsigned int>();
    rhs.header.stamp = node["header/stamp"].as<unsigned long>();
    return true;
}
}
