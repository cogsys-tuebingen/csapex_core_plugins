#ifndef KEY_VALUE_MESSAGE_H
#define KEY_VALUE_MESSAGE_H
#include <csapex/msg/message_template.hpp>
#include <csapex_core_plugins/csapex_core_lib_export.h>


namespace csapex {
namespace connection_types {

struct CSAPEX_CORE_LIB_EXPORT KeyValueMessage : public MessageTemplate<
        std::pair<std::string, TokenData::ConstPtr>,
        KeyValueMessage>
{

public:
    KeyValueMessage(const std::string& frame_id = "/", Stamp stamp = 0);
    KeyValueMessage(std::string name, TokenData::ConstPtr msg, const std::string& frame_id = "/", Stamp stamp = 0);
};


/// TRAITS
template <>
struct CSAPEX_CORE_LIB_EXPORT type<KeyValueMessage> {
    static std::string name() {
        return "KeyValue";
    }
};

//SerializationBuffer& operator << (SerializationBuffer& data, const KeyValueMessage& t);
//const SerializationBuffer& operator >> (const SerializationBuffer& data, KeyValueMessage& t);

}

template <>
inline CSAPEX_CORE_LIB_EXPORT std::shared_ptr<connection_types::KeyValueMessage> makeEmpty<connection_types::KeyValueMessage>()
{
    return std::shared_ptr<connection_types::KeyValueMessage>(new connection_types::KeyValueMessage);
}

}

/// YAML
namespace YAML {
template<>
struct CSAPEX_CORE_LIB_EXPORT convert<csapex::connection_types::KeyValueMessage> {
  static Node encode(const csapex::connection_types::KeyValueMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::KeyValueMessage& rhs);
};
}
#endif // KEY_VALUE_MESSAGE_H
