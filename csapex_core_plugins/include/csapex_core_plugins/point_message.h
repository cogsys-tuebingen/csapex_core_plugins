#ifndef POINT_MESSAGE_H
#define POINT_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>


namespace csapex {
namespace connection_types {

struct PointMessage : public Message
{
public:
    typedef boost::shared_ptr<PointMessage> Ptr;

    PointMessage();

    static ConnectionType::Ptr make();

    virtual ConnectionType::Ptr clone();
    virtual ConnectionType::Ptr toType();

public:
    float x;
    float y;
};


/// TRAITS
template <>
struct type<PointMessage> {
    static std::string name() {
        return "Point";
    }
};

}
}

/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::PointMessage> {
  static Node encode(const csapex::connection_types::PointMessage& rhs);
  static bool decode(const Node& node, csapex::connection_types::PointMessage& rhs);
};
}

#endif // POINT_MESSAGE_H
