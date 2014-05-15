#ifndef DESCRIPTOR_MESSAGE_H
#define DESCRIPTOR_MESSAGE_H

/// COMPONENT
#include <csapex_scan_2d/labeled_scan.h>

/// PROJECT
#include <csapex/model/message.h>

namespace csapex {
namespace connection_types {


struct LabeledScanMessage : public MessageTemplate<LabeledScan, LabeledScanMessage>
{
    LabeledScanMessage();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // DESCRIPTOR_MESSAGE_H
