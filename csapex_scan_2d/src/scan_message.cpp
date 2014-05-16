/// HEADER
#include <csapex_scan_2d/scan_message.h>

using namespace csapex;
using namespace connection_types;


ScanMessage::ScanMessage()
    : MessageTemplate<Scan, ScanMessage> ("Scan")
{}

void ScanMessage::writeYaml(YAML::Emitter &yaml)
{
    yaml << YAML::Flow << YAML::Key << "ranges" << YAML::Value;
    yaml << YAML::BeginSeq;
    std::vector<float>::const_iterator r = value.ranges.begin();
    for(; r != value.ranges.end(); ++r) {
        yaml << *r;
    }
    yaml << YAML::EndSeq;
}

void ScanMessage::readYaml(const YAML::Node &node)
{
}
