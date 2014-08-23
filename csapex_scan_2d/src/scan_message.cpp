/// HEADER
#include <csapex_scan_2d/scan_message.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <utils_laser_processing/common/yaml-io.hpp>

using namespace csapex;
using namespace connection_types;
using namespace lib_laser_processing;

ScanMessage::ScanMessage()
    : MessageTemplate<lib_laser_processing::Scan, ScanMessage> ("/")
{}



/// YAML
namespace YAML {
Node convert<csapex::connection_types::ScanMessage>::encode(const csapex::connection_types::ScanMessage& rhs)
{
    Node node;

    node["value"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::ScanMessage>::decode(const Node& node, csapex::connection_types::ScanMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }

    rhs.value = node.as<Scan>();
    return true;
}
}
