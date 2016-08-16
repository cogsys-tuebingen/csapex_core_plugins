#ifndef POINT_MESSAGE_H
#define POINT_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>


namespace csapex {
namespace connection_types {

struct CSAPEX_EXPORT_PLUGIN PointMessage : public Message
{
public:
    typedef std::shared_ptr<PointMessage> Ptr;

    PointMessage(Message::Stamp stamp_micro_seconds = 0);

    virtual TokenData::Ptr clone() const override;
    virtual TokenData::Ptr toType() const override;

public:
    float x;
    float y;
};


/// TRAITS
template <>
struct CSAPEX_EXPORT_PLUGIN type<PointMessage> {
    static std::string name() {
        return "Point";
    }
};

template <>
inline CSAPEX_EXPORT_PLUGIN std::shared_ptr<PointMessage> makeEmpty<PointMessage>()
{
    return std::shared_ptr<PointMessage>(new PointMessage(0));
}

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
