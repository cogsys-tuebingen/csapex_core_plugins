/// HEADER
#include <csapex_transform/transform_message.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <tf/transform_datatypes.h>

using namespace csapex;
using namespace connection_types;

TransformMessage::TransformMessage(const std::string& from_frame, const std::string& to_frame)
    : MessageTemplate<tf::Transform, TransformMessage> (from_frame), child_frame(to_frame)
{}

TransformMessage::TransformMessage()
    : MessageTemplate<tf::Transform, TransformMessage> ("/"), child_frame("/")
{}

ConnectionType::Ptr TransformMessage::clone() {
    Ptr new_msg(new TransformMessage(frame_id, child_frame));
    new_msg->value = value;
    return new_msg;
}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::TransformMessage>::encode(const csapex::connection_types::TransformMessage& rhs)
{
    const tf::Quaternion& q = rhs.value.getRotation();
    const tf::Vector3& t = rhs.value.getOrigin();

    Node node;
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
    if(!node.IsMap()) {
        return false;
    }

    std::vector<float> o = node["orientation"].as< std::vector<float> >();
    std::vector<float> t = node["orientation"].as< std::vector<float> >();

    if(o.size() != 4 || t.size() != 3) {
        return false;
    }

    rhs.value = tf::Transform(tf::Quaternion(o[0],o[1],o[2],o[3]), tf::Vector3(t[0], t[1], t[2]));

    return true;
}
}
