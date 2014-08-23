/// HEADER
#include <csapex_scan_2d/labeled_scan_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <utils_laser_processing/common/yaml-io.hpp>

using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;


LabeledScanMessage::LabeledScanMessage()
    : MessageTemplate<LabeledScan, LabeledScanMessage> ("/")
{}

/// YAML
namespace YAML {
Node convert<csapex::connection_types::LabeledScanMessage>::encode(const csapex::connection_types::LabeledScanMessage& rhs)
{
    Node node;

    node["value"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::LabeledScanMessage>::decode(const Node& node, csapex::connection_types::LabeledScanMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    rhs.value = node.as<LabeledScan>();
    return true;
}
}


//void LabeledScanMessage::writeYaml(YAML::Emitter &yaml) const
//{
//    yaml << YAML::Flow << YAML::Key << "ranges" << YAML::Value;
//    yaml << YAML::BeginSeq;
//    std::vector<LaserBeam>::const_iterator r = value.rays.begin();
//    std::vector<int>::const_iterator l = value.labels.begin();
//    for(; r != value.rays.end(); ++r, ++l) {
//        yaml << YAML::Flow << YAML::BeginSeq << r->yaw << r->range << *l << YAML::EndSeq;
//    }
//    yaml << YAML::EndSeq;
//}

//void LabeledScanMessage::readYaml(const YAML::Node &doc)
//{
//    if(!YAML::exists(doc, "ranges")) {
//        return;
//    }

//    const YAML::Node& node = doc["ranges"];
//    apex_assert_hard(node.Type() == YAML::NodeType::Sequence);

//    std::size_t count = node.size(); // infs and nans cause problems?
//    value.rays.resize(count);
//    value.labels.resize(count);

//    for(std::size_t i = 0; i< count; ++i) {
//        const YAML::Node& ray = node[i];

//        double yaw = 0;
//        ray[0] >> yaw;

//        double range = 0;
//        std::string range_s;
//        ray[1] >> range_s;
//        if(range_s == "inf") {
//            range = std::numeric_limits<double>::infinity();
//        } else {
//            std::stringstream converter;
//            converter.str(range_s);
//            converter >> range;
//        }

//        value.rays[i] = LaserBeam(yaw, range);

//        int label;
//        ray[2] >> label;
//        value.labels[i] = label;
//    }
//}
