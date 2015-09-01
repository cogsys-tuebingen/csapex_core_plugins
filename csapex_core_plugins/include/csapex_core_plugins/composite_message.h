#ifndef COMPOSITE_MESSAGE_H
#define COMPOSITE_MESSAGE_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/msg/message_traits.h>

/// SYSTEM
#include <string>
#include <boost/static_assert.hpp>
#include <vector>

namespace csapex {
namespace connection_types {

struct CompositeMessage : public Message
{
    typedef std::shared_ptr<CompositeMessage> Ptr;
    typedef std::shared_ptr<const CompositeMessage> ConstPtr;

    CompositeMessage(const std::string& frame_id = "/", Stamp stamp_micro_seconds = 0);

    ConnectionType::Ptr getSubType() const;

    template <typename T>
    static void registerType()
    {
        // not used for this type
    }


    template <typename T>
    static CompositeMessage::Ptr make()
    {
        return CompositeMessage::Ptr (new CompositeMessage(connection_types::makeEmpty<T>(), "/", 0));
    }

    static CompositeMessage::Ptr make(ConnectionType::Ptr type)
    {
        return CompositeMessage::Ptr (new CompositeMessage(type->toType(), "/", 0));
    }

    static CompositeMessage::Ptr make();

    virtual ConnectionType::Ptr clone() const override;
    virtual ConnectionType::Ptr toType() const override;

    virtual bool canConnectTo(const ConnectionType* other_side) const override;
    virtual bool acceptsConnectionFrom(const ConnectionType *other_side) const override;

private:
    CompositeMessage(ConnectionType::Ptr type, const std::string& frame_id, Stamp stamp_micro_seconds);

public:
    std::vector<ConnectionType::ConstPtr> value;

private:
    ConnectionType::Ptr type_;

};

template <>
struct type<CompositeMessage> {
    static std::string name() {
        return "MessageComposite";
    }
};

}
}


/// YAML
namespace YAML {
template<>
struct convert<csapex::connection_types::CompositeMessage> {
    static Node encode(const csapex::connection_types::CompositeMessage& rhs);
    static bool decode(const Node& node, csapex::connection_types::CompositeMessage& rhs);
};
}

#endif // COMPOSITE_MESSAGE_H

