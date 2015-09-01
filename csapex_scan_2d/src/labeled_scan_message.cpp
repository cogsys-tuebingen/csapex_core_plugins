/// HEADER
#include <csapex_scan_2d/labeled_scan_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <utils_laser_processing/common/yaml-io.hpp>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::LabeledScanMessage)

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
    Node node = convert<csapex::connection_types::Message>::encode(rhs);

    node["value"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::LabeledScanMessage>::decode(const Node& node, csapex::connection_types::LabeledScanMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.value = node["value"].as<LabeledScan>();
    return true;
}
}
