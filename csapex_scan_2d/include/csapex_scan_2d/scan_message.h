#ifndef SCAN_MESSAGE_H
#define SCAN_MESSAGE_H

/// COMPONENT
#include <csapex_scan_2d/scan.h>

/// PROJECT
#include <csapex/model/message.h>

namespace csapex {
namespace connection_types {


struct ScanMessage : public MessageTemplate<Scan, ScanMessage>
{
    ScanMessage();

    void writeYaml(YAML::Emitter& yaml);
    void readYaml(const YAML::Node& node);
};

}
}

#endif // SCAN_MESSAGE_H
