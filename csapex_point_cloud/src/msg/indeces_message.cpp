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

