/// HEADER
#include <csapex_core_plugins/composite_message.h>

/// COMPONENT
#include <csapex/msg/message_traits.h>
#include <csapex/utility/register_msg.h>
#include <csapex/serialization/yaml.h>
#include <csapex/msg/any_message.h>

CSAPEX_REGISTER_MESSAGE(csapex::connection_types::CompositeMessage)

using namespace csapex;
using namespace connection_types;


CompositeMessage::CompositeMessage(const std::string& frame_id, Message::Stamp stamp)
    : Message ("MessageComposite", frame_id, stamp)
{
    type_ = connection_types::makeEmpty<AnyMessage>();
}
CompositeMessage::CompositeMessage(ConnectionType::Ptr type, const std::string& frame_id, Message::Stamp stamp)
    : Message ("MessageComposite", frame_id, stamp)
{
    setDescriptiveName("Composite");
    type_ = type;
}

ConnectionType::Ptr CompositeMessage::getSubType() const
{
    return type_;
}

CompositeMessage::Ptr CompositeMessage::make(){
    Ptr new_msg(new CompositeMessage("/"));
    return new_msg;
}

ConnectionType::Ptr CompositeMessage::clone() const
{
    Ptr new_msg(new CompositeMessage(frame_id));
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr CompositeMessage::toType() const
{
    return make();
}

bool CompositeMessage::canConnectTo(const ConnectionType *other_side) const
{
    const CompositeMessage* vec = dynamic_cast<const CompositeMessage*> (other_side);
    if(vec != 0 && type_->canConnectTo(vec->getSubType().get())) {
        return true;
    } else {
        return other_side->canConnectTo(this);
    }
}

bool CompositeMessage::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    const CompositeMessage* vec = dynamic_cast<const CompositeMessage*> (other_side);
    if(vec != 0 && type_->acceptsConnectionFrom(vec->getSubType().get())) {
        return true;
    } else {
        return other_side->acceptsConnectionFrom(this);
    }
}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::CompositeMessage>::encode(const csapex::connection_types::CompositeMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["values"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::CompositeMessage>::decode(const Node& node, csapex::connection_types::CompositeMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.value = node["values"].as<std::vector<ConnectionTypeConstPtr>>();
    return true;
}
}

