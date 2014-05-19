#ifndef INDECE_MESSAGE_H
#define INDECE_MESSAGE_H

/// PROJECT
#include <csapex/model/message.h>

/// SYSTEM
#include <pcl/PointIndices.h>

namespace csapex {
namespace connection_types {
class PointIndecesMessage : public MessageTemplate<pcl::PointIndices::Ptr, PointIndecesMessage>
{
public:
    PointIndecesMessage();

    virtual void writeYaml(YAML::Emitter &yaml);
    virtual void readYaml(const YAML::Node &node);

};
}
}
#endif // INDECE_MESSAGE_H
