/// HEADER
#include <csapex_scan_2d/labeled_scan_message.h>

using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;


LabeledScanMessage::LabeledScanMessage()
    : MessageTemplate<LabeledScan, LabeledScanMessage> ("LabeledScan")
{}

void LabeledScanMessage::writeYaml(YAML::Emitter &yaml) const
{
    yaml << YAML::Flow << YAML::Key << "ranges" << YAML::Value;
    yaml << YAML::BeginSeq;
    std::vector<LaserBeam>::const_iterator r = value.rays.begin();
    std::vector<int>::const_iterator l = value.labels.begin();
    for(; r != value.rays.end(); ++r, ++l) {
        yaml << r->range << *l;
    }
    yaml << YAML::EndSeq;
}

void LabeledScanMessage::readYaml(const YAML::Node &node)
{
}
