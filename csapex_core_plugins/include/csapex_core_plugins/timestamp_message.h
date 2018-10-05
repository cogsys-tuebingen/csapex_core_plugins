#ifndef TIMESTAMP_MESSAGE_H
#define TIMESTAMP_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>
#include <csapex_core_plugins/csapex_core_lib_export.h>

/// SYSTEM
#include <chrono>

namespace csapex
{
namespace connection_types
{
struct CSAPEX_CORE_LIB_EXPORT TimestampMessage : public MessageTemplate<std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds>, TimestampMessage>
{
public:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds> Tp;

public:
    TimestampMessage(Tp time = Tp(std::chrono::microseconds(0)));
};

/// TRAITS
template <>
struct CSAPEX_CORE_LIB_EXPORT type<TimestampMessage>
{
    static std::string name()
    {
        return "Timestamp";
    }
};

}  // namespace connection_types

template <>
inline CSAPEX_CORE_LIB_EXPORT std::shared_ptr<connection_types::TimestampMessage> makeEmpty<connection_types::TimestampMessage>()
{
    return std::shared_ptr<connection_types::TimestampMessage>(new connection_types::TimestampMessage());
}
/// SERIALIZATION
SerializationBuffer& operator<<(SerializationBuffer& data, const std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds>& t);
const SerializationBuffer& operator>>(const SerializationBuffer& data, std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds>& t);
}  // namespace csapex

/// YAML
namespace YAML
{
template <>
struct CSAPEX_CORE_LIB_EXPORT convert<csapex::connection_types::TimestampMessage>
{
    static Node encode(const csapex::connection_types::TimestampMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::TimestampMessage& rhs);
};
}  // namespace YAML

#endif  // DURATION_MESSAGE_H
