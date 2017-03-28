#ifndef INDICES_MESSAGE_H
#define INDICES_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <pcl/PointIndices.h>

namespace csapex {
namespace connection_types {
class PointIndicesMessage : public MessageTemplate<pcl::PointIndices::Ptr, PointIndicesMessage>
{
public:
    PointIndicesMessage();
};


/// TRAITS
template <>
struct type<PointIndicesMessage> {
    static std::string name() {
        return "PointIndices";
    }
};
}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::PointIndicesMessage> {
  static Node encode(const csapex::connection_types::PointIndicesMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::PointIndicesMessage& rhs);
};
template<>
struct convert<pcl::PointIndices> {
  static Node encode(const pcl::PointIndices& rhs);
  static bool decode(const Node& node, pcl::PointIndices& rhs);
};
}
#endif // INDICES_MESSAGE_H
