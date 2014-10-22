/// HEADER
#include <csapex_core_plugins/vector_message.h>

/// COMPONENT
#include <csapex/msg/message_traits.h>
#include <csapex/utility/register_msg.h>

CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::GenericVectorMessage, g_instance_generic_vector_)
CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::VectorMessage, g_instance_vector_)

using namespace csapex;
using namespace connection_types;

const GenericVectorMessage::EntryInterface::Ptr GenericVectorMessage::EntryInterface::NullPtr;

GenericVectorMessage::GenericVectorMessage(EntryInterface::Ptr impl, const std::string& frame_id, Message::Stamp stamp)
    : Message (type<GenericVectorMessage>::name(), frame_id, stamp), impl(impl)
{
}

ConnectionType::Ptr GenericVectorMessage::clone() {
    Ptr new_msg(new GenericVectorMessage(impl->cloneEntry(), frame_id, impl->stamp));
    return new_msg;
}

ConnectionType::Ptr GenericVectorMessage::toType() {
    Ptr new_msg(new GenericVectorMessage(impl->cloneEntry(), frame_id, 0));
    return new_msg;
}

bool GenericVectorMessage::canConnectTo(const ConnectionType *other_side) const
{
    return impl->canConnectTo(other_side);
}

bool GenericVectorMessage::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    return impl->acceptsConnectionFrom(other_side);
}




/// YAML
namespace YAML {
Node convert<csapex::connection_types::GenericVectorMessage>::encode(const csapex::connection_types::GenericVectorMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    rhs.encode(node);
    return node;
}

bool convert<csapex::connection_types::GenericVectorMessage>::decode(const Node& node, csapex::connection_types::GenericVectorMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    rhs.decode(node);
    return true;
}
}



//// OLD
VectorMessage::VectorMessage(const std::string& frame_id, Message::Stamp stamp)
    : Message (type<VectorMessage>::name(), frame_id, stamp)
{
    type_ = connection_types::makeEmpty<AnyMessage>();
}
VectorMessage::VectorMessage(ConnectionType::Ptr type, const std::string& frame_id, Message::Stamp stamp)
    : Message (std::string("std::vector<") + type->rawName()  + "::Ptr>", frame_id, stamp)
{
    type_ = type;
}

ConnectionType::Ptr VectorMessage::getSubType() const
{
    return type_;
}

VectorMessage::Ptr VectorMessage::make(){
    Ptr new_msg(new VectorMessage("/"));
    return new_msg;
}

ConnectionType::Ptr VectorMessage::clone() {
    Ptr new_msg(new VectorMessage(frame_id));
    new_msg->value = value;
    return new_msg;
}

ConnectionType::Ptr VectorMessage::toType() {
    return make();
}

bool VectorMessage::canConnectTo(const ConnectionType *other_side) const
{
    const VectorMessage* vec = dynamic_cast<const VectorMessage*> (other_side);
    if(vec != 0 && type_->canConnectTo(vec->getSubType().get())) {
        return true;
    } else {
        return other_side->canConnectTo(this);
    }
}

bool VectorMessage::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    const VectorMessage* vec = dynamic_cast<const VectorMessage*> (other_side);
    if(vec != 0 && type_->acceptsConnectionFrom(vec->getSubType().get())) {
        return true;
    } else {
        return other_side->acceptsConnectionFrom(this);
    }
}


/// YAML
namespace YAML {
Node convert<csapex::connection_types::VectorMessage>::encode(const csapex::connection_types::VectorMessage& rhs)
{
    Node node = convert<csapex::connection_types::Message>::encode(rhs);
    node["values"] = rhs.value;
    return node;
}

bool convert<csapex::connection_types::VectorMessage>::decode(const Node& node, csapex::connection_types::VectorMessage& rhs)
{
    if(!node.IsMap()) {
        return false;
    }
    convert<csapex::connection_types::Message>::decode(node, rhs);
    return true;
}
}

namespace YAML {
Node convert<csapex::ConnectionType>::encode(const csapex::ConnectionType& rhs)
{
    return MessageFactory::serializeMessage(rhs);
}

bool convert<csapex::ConnectionType>::decode(const Node& node, csapex::ConnectionType& rhs)
{
    rhs = *MessageFactory::deserializeMessage(node);
    return true;
}
}
