/// HEADER
#include <csapex_scan_2d/labeled_scan_message.h>

using namespace csapex;
using namespace connection_types;


LabeledScanMessage::LabeledScanMessage()
    : MessageTemplate<LabeledScan, LabeledScanMessage> ("LabeledScan")
{}

void LabeledScanMessage::writeYaml(YAML::Emitter &yaml)
{
    yaml << YAML::Flow << YAML::Key << "ranges" << YAML::Value;
    yaml << YAML::BeginSeq;
    std::vector<float>::const_iterator r = value.ranges.begin();
    std::vector<int>::const_iterator l = value.labels.begin();
    for(; r != value.ranges.end(); ++r, ++l) {
        yaml << *r << *l;
    }
    yaml << YAML::EndSeq;
}

void LabeledScanMessage::readYaml(const YAML::Node &node)
{
}
