/// HEADER
#include <csapex_point_cloud/point_cloud_message.h>

using namespace csapex;
using namespace connection_types;

PointCloudMessage::PointCloudMessage(const std::string& frame_id)
    : Message (type<PointCloudMessage>::name(), frame_id)
{
}

ConnectionType::Ptr PointCloudMessage::clone() {
    Ptr new_msg(new PointCloudMessage(frame_id));
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr PointCloudMessage::toType() {
    Ptr new_msg(new PointCloudMessage("/"));
    return new_msg;
}

ConnectionType::Ptr PointCloudMessage::make(){
    Ptr new_msg(new PointCloudMessage("/"));
    return new_msg;
}


std::string PointCloudMessage::name() const
{
    return Message::name();
}

bool PointCloudMessage::acceptsConnectionFrom(const ConnectionType* other_side) const {
    return dynamic_cast<const PointCloudMessage*> (other_side);
}

void PointCloudMessage::writeYaml(YAML::Emitter& yaml) const {
    yaml << YAML::Key << "value" << YAML::Value << "not implemented";
}
void PointCloudMessage::readYaml(const YAML::Node&) {
}

PointCloudMessage::PointCloudMessage()
    : Message (type<PointCloudMessage>::name(), "/")
{
}
