/// HEADER
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/utility/register_msg.h>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <tf/transform_datatypes.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::TransformMessage)

using namespace csapex;
using namespace connection_types;

TransformMessage::TransformMessage(const std::string& frame_id, const std::string& child_frame_id) : MessageTemplate<tf::Transform, TransformMessage>(frame_id), child_frame(child_frame_id)
{
    sanitize();
}

TransformMessage::TransformMessage() : MessageTemplate<tf::Transform, TransformMessage>(""), child_frame("")
{
}

void TransformMessage::sanitize()
{
    if (frame_id.size() > 0 && frame_id.at(0) == '/') {
        frame_id = frame_id.substr(1);
    }
    if (child_frame.size() > 0 && child_frame.at(0) == '/') {
        child_frame = child_frame.substr(1);
    }
}

bool TransformMessage::cloneData(const TransformMessage& other)
{
    if (!Message::cloneDataFrom(other)) {
        return false;
    }

    child_frame = other.child_frame;
    value = other.value;

    return true;
}

/// YAML
namespace YAML
{
Node convert<csapex::connection_types::TransformMessage>::encode(const csapex::connection_types::TransformMessage& rhs)
{
    const tf::Quaternion& q = rhs.value.getRotation();
    const tf::Vector3& t = rhs.value.getOrigin();

    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["child_frame"] = rhs.child_frame;

    node["orientation"].push_back(q.x());
    node["orientation"].push_back(q.y());
    node["orientation"].push_back(q.z());
    node["orientation"].push_back(q.w());
    node["translation"].push_back(t.x());
    node["translation"].push_back(t.y());
    node["translation"].push_back(t.z());
    return node;
}

bool convert<csapex::connection_types::TransformMessage>::decode(const Node& node, csapex::connection_types::TransformMessage& rhs)
{
    if (!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);

    if (node["child_frame"].IsDefined()) {
        rhs.child_frame = node["child_frame"].as<std::string>();
    }

    std::vector<float> o = node["orientation"].as<std::vector<float>>();
    std::vector<float> t = node["translation"].as<std::vector<float>>();

    if (o.size() != 4 || t.size() != 3) {
        return false;
    }

    rhs.value = tf::Transform(tf::Quaternion(o[0], o[1], o[2], o[3]), tf::Vector3(t[0], t[1], t[2]));
    rhs.sanitize();

    return true;
}
}  // namespace YAML
