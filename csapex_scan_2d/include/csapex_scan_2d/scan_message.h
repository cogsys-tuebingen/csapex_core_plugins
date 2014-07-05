#ifndef SCAN_MESSAGE_H
#define SCAN_MESSAGE_H

/// COMPONENT
#include <utils_laser_processing/data/scan.h>

/// PROJECT
#include <csapex/model/message.h>

namespace csapex {
namespace connection_types {


struct ScanMessage : public MessageTemplate<lib_laser_processing::Scan, ScanMessage>
{
    ScanMessage();

    void writeYaml(YAML::Emitter& yaml) const;
    void readYaml(const YAML::Node& node);
};

/// TRAITS
template <>
struct type<ScanMessage> {
    static std::string name() {
        return "Scan";
    }
};

}
}

#endif // SCAN_MESSAGE_H
