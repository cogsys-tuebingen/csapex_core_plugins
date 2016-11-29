#ifndef MAP_MESSAGE_H
#define MAP_MESSAGE_H
/// COMPONENTS
#include <csapex/msg/message_template.hpp>
#include <csapex_core_plugins/csapex_core_lib_export.h>
#include <csapex_core_plugins/key_value_message.h>
/// SYSTEM
#include <vector>

namespace csapex {
namespace connection_types {

struct CSAPEX_CORE_LIB_EXPORT MapMessage : public MessageTemplate<
        std::vector<KeyValueMessage>,
        MapMessage>
{

public:
    MapMessage(std::size_t size = 0, const std::string& frame_id = "/", Stamp stamp = 0);
};


/// TRAITS
template <>
struct CSAPEX_CORE_LIB_EXPORT type<MapMessage> {
    static std::string name() {
        return "Map";
    }
};

template <>
inline CSAPEX_CORE_LIB_EXPORT std::shared_ptr<MapMessage> makeEmpty<MapMessage>()
{
    return std::shared_ptr<MapMessage>(new MapMessage());
}

}
}

/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_LIB_EXPORT convert<csapex::connection_types::MapMessage> {
  static Node encode(const csapex::connection_types::MapMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::MapMessage& rhs);
};
}
#endif // MAP_MESSAGE_H
