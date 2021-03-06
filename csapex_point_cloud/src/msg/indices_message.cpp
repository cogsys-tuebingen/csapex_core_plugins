/// HEADER
#include <csapex_point_cloud/msg/indices_message.h>

/// PROJECT
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::PointIndicesMessage)

using namespace csapex::connection_types;

PointIndicesMessage::PointIndicesMessage() : MessageTemplate<pcl::PointIndices::Ptr, PointIndicesMessage>("/")
{
    value.reset(new pcl::PointIndices);
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::PointIndicesMessage>::encode(const csapex::connection_types::PointIndicesMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["indices"] = *rhs.value;
    return node;
}

bool convert<csapex::connection_types::PointIndicesMessage>::decode(const Node& node, csapex::connection_types::PointIndicesMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.value.reset(new pcl::PointIndices);
    *rhs.value = node["indices"].as<pcl::PointIndices>();
    return true;
}

Node convert<pcl::PointIndices>::encode(const pcl::PointIndices& rhs)
{
    Node node;
    node["indices"] = rhs.indices;
    node["header/frame_id"] = rhs.header.frame_id;
    node["header/seq"] = rhs.header.seq;
    node["header/stamp"] = rhs.header.stamp;

    return node;
}

bool convert<pcl::PointIndices>::decode(const Node& node, pcl::PointIndices& rhs)
{
    if (!node.IsMap()) {
        return false;
    }
    rhs.indices = node["indices"].as<std::vector<int>>();
    rhs.header.frame_id = node["header/frame_id"].as<std::string>();
    rhs.header.seq = node["header/seq"].as<unsigned int>();
    rhs.header.stamp = node["header/stamp"].as<unsigned long>();
    return true;
}
}  // namespace YAML
