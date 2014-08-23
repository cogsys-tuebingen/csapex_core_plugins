#ifndef DESCRIPTOR_MESSAGE_H
#define DESCRIPTOR_MESSAGE_H

/// COMPONENT
#include <utils_laser_processing/data/labeled_scan.h>

/// PROJECT
#include <csapex/msg/message_template.hpp>

namespace csapex {
namespace connection_types {


struct LabeledScanMessage : public MessageTemplate<lib_laser_processing::LabeledScan, LabeledScanMessage>
{
    LabeledScanMessage();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
};

/// TRAITS
template <>
struct type<LabeledScanMessage> {
    static std::string name() {
        return "LabeledScan";
    }
};
}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::LabeledScanMessage> {
  static Node encode(const csapex::connection_types::LabeledScanMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::LabeledScanMessage& rhs);
};
}


#endif // DESCRIPTOR_MESSAGE_H
