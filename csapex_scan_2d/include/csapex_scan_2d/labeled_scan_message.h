#ifndef DESCRIPTOR_MESSAGE_H
#define DESCRIPTOR_MESSAGE_H

/// COMPONENT
#include <utils_laser_processing/data/labeled_scan.h>

/// PROJECT
#include <csapex/model/message.h>

namespace csapex {
namespace connection_types {


struct LabeledScanMessage : public MessageTemplate<lib_laser_processing::LabeledScan, LabeledScanMessage>
{
    LabeledScanMessage();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
};

}
}

#endif // DESCRIPTOR_MESSAGE_H
