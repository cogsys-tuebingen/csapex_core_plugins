#ifndef TIMESTAMP_MESSAGE_H
#define TIMESTAMP_MESSAGE_H

/// PROJECT
#include <csapex/msg/message_template.hpp>

/// SYSTEM
#include <chrono>

namespace csapex {
namespace connection_types {

struct TimestampMessage : public MessageTemplate<
        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds>,
        TimestampMessage>
{
public:
    typedef std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::microseconds> Tp;

public:
    TimestampMessage(Tp time = Tp(std::chrono::microseconds(0)), Message::Stamp stamp_micro_seconds = 0);
};


/// TRAITS
template <>
struct type<TimestampMessage> {
    static std::string name() {
        return "Timestamp";
    }
};

template <>
inline std::shared_ptr<TimestampMessage> makeEmpty<TimestampMessage>()
{
    return std::shared_ptr<TimestampMessage>(new TimestampMessage());
}

}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::TimestampMessage> {
  static Node encode(const csapex::connection_types::TimestampMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::TimestampMessage& rhs);
};
}

#endif // DURATION_MESSAGE_H
