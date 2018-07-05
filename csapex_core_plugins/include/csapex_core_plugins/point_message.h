#ifndef POINT_MESSAGE_H
#define POINT_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex_core_plugins/csapex_core_lib_export.h>


namespace YAML
{
template<typename T, typename S>
struct as_if;
}

namespace csapex {
namespace connection_types {

struct CSAPEX_CORE_LIB_EXPORT PointMessage : public Message
{
protected:
    CLONABLE_IMPLEMENTATION(PointMessage);

    friend class YAML::as_if<PointMessage, void>;

public:
    typedef std::shared_ptr<PointMessage> Ptr;

    PointMessage(Message::Stamp stamp_micro_seconds);

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

private:
    PointMessage();

public:
    float x;
    float y;
};


/// TRAITS
template <>
struct CSAPEX_CORE_LIB_EXPORT type<PointMessage> {
    static std::string name() {
        return "Point";
    }
};

}

template <>
inline CSAPEX_CORE_LIB_EXPORT std::shared_ptr<connection_types::PointMessage> makeEmpty<connection_types::PointMessage>()
{
    return std::shared_ptr<connection_types::PointMessage>(new connection_types::PointMessage(0));
}
}

/// YAML
namespace YAML {
template<>
struct CSAPEX_EXPORT_PLUGIN convert<csapex::connection_types::PointMessage> {
  static Node encode(const csapex::connection_types::PointMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::PointMessage& rhs);
};
}

#endif // POINT_MESSAGE_H
