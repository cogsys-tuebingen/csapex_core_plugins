#ifndef INDECE_MESSAGE_H
#define INDECE_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <pcl/PointIndices.h>

namespace csapex {
namespace connection_types {
class PointIndecesMessage : public MessageTemplate<pcl::PointIndices::Ptr, PointIndecesMessage>
{
public:
    PointIndecesMessage();

    virtual void writeYaml(YAML::Emitter &yaml) const;
    virtual void readYaml(const YAML::Node &node);

};


/// TRAITS
template <>
struct type<PointIndecesMessage> {
    static std::string name() {
        return "PointIndeces";
    }
};
}
}
#endif // INDECE_MESSAGE_H
