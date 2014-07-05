/// HEADER
#include <csapex_transform/duration_message.h>

/// SYSTEM
#include <boost/date_time.hpp>

using namespace csapex;
using namespace connection_types;

DurationMessage::DurationMessage()
    : MessageTemplate<ros::Duration, DurationMessage> ("/")
{}

void DurationMessage::writeYaml(YAML::Emitter& yaml) const {
    yaml << YAML::Key << "duration" << YAML::Value << value.toNSec() << YAML::EndSeq;;
}

void DurationMessage::readYaml(const YAML::Node& node) {
    if(exists(node, "duration")) {
        u_int64_t nano;
        node["duration"] >> nano;
        value = value.fromNSec(nano);
    }
}
