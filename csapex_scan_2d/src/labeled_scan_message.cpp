/// HEADER
#include <csapex_scan_2d/labeled_scan_message.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;


LabeledScanMessage::LabeledScanMessage()
    : MessageTemplate<LabeledScan, LabeledScanMessage> ("/")
{}

void LabeledScanMessage::writeYaml(YAML::Emitter &yaml) const
{
    yaml << YAML::Flow << YAML::Key << "ranges" << YAML::Value;
    yaml << YAML::BeginSeq;
    std::vector<LaserBeam>::const_iterator r = value.rays.begin();
    std::vector<int>::const_iterator l = value.labels.begin();
    for(; r != value.rays.end(); ++r, ++l) {
        yaml << r->yaw << r->range << *l;
    }
    yaml << YAML::EndSeq;
}

void LabeledScanMessage::readYaml(const YAML::Node &doc)
{
    if(!YAML::exists(doc, "ranges")) {
        return;
    }

    const YAML::Node& node = doc["ranges"];
    apex_assert_hard(node.Type() == YAML::NodeType::Sequence);

    std::size_t count = node.size(); // infs and nans cause problems?
    value.rays.resize(count / 3);
    value.labels.resize(count / 3);

    for(std::size_t i = 0, j = 0; j < count; ++i) {
        double range = 0;
        node[j++] >> range;

        double yaw = 0;
        node[j++] >> yaw;

        value.rays[i] = LaserBeam(yaw, range);

        int label;
        node[j++] >> label;
        value.labels[i] = label;
    }
}
