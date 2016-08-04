/// HEADER
#include <csapex_core_plugins/vector_message.h>

/// COMPONENT
#include <csapex/msg/token_traits.h>
#include <csapex/utility/register_msg.h>
#include <csapex/serialization/yaml.h>
#include <csapex/msg/no_message.h>

CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::GenericVectorMessage, g_instance_generic_vector_)
CSAPEX_REGISTER_MESSAGE_WITH_NAME(csapex::connection_types::VectorMessage, g_instance_vector_)

using namespace csapex;
using namespace connection_types;

GenericVectorMessage::GenericVectorMessage(EntryInterface::Ptr impl, const std::string& frame_id, Message::Stamp stamp)
    : Message (type<GenericVectorMessage>::name(), frame_id, stamp), impl(impl)
{
}

TokenData::Ptr GenericVectorMessage::clone() const
{
    Ptr new_msg(new GenericVectorMessage(impl->cloneEntry(), frame_id, impl->stamp_micro_seconds));
    return new_msg;
}

TokenData::Ptr GenericVectorMessage::toType() const
{
    Ptr new_msg(new GenericVectorMessage(impl->cloneEntry(), frame_id, 0));
    return new_msg;
}

bool GenericVectorMessage::canConnectTo(const TokenData *other_side) const
{
    return impl->canConnectTo(other_side);
}

bool GenericVectorMessage::acceptsConnectionFrom(const TokenData *other_side) const
{
    return impl->acceptsConnectionFrom(other_side);
}

std::string GenericVectorMessage::descriptiveName() const
{
    return impl->descriptiveName();
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
    : Message ("MessageVector", frame_id, stamp)
{
    type_ = connection_types::makeEmpty<AnyMessage>();
}
VectorMessage::VectorMessage(TokenData::Ptr type, const std::string& frame_id, Message::Stamp stamp)
    : Message ("MessageVector", frame_id, stamp)
{
    setDescriptiveName(std::string("std::vector<") + type->typeName()  + "::Ptr>");
    type_ = type;
}

TokenData::Ptr VectorMessage::getSubType() const
{
    return type_;
}

VectorMessage::Ptr VectorMessage::make(){
    Ptr new_msg(new VectorMessage("/"));
    return new_msg;
}

TokenData::Ptr VectorMessage::clone() const
{
    Ptr new_msg(new VectorMessage(type_, frame_id, stamp_micro_seconds));
    new_msg->value = value;
    return new_msg;
}

TokenData::Ptr VectorMessage::toType() const
{
    return make();
}

bool VectorMessage::canConnectTo(const TokenData *other_side) const
{
    const VectorMessage* vec = dynamic_cast<const VectorMessage*> (other_side);
    if(vec != 0 && type_->canConnectTo(vec->getSubType().get())) {
        return true;
    } else {
        return other_side->canConnectTo(this);
    }
}

bool VectorMessage::acceptsConnectionFrom(const TokenData *other_side) const
{
    const VectorMessage* vec = dynamic_cast<const VectorMessage*> (other_side);
    if(vec != 0 && type_->acceptsConnectionFrom(vec->getSubType().get())) {
        return true;
    } else {
        return other_side->acceptsConnectionFrom(this);
    }
}

bool VectorMessage::isContainer() const
{
    return true;
}

TokenData::Ptr VectorMessage::nestedType() const
{
    return value.empty() ? connection_types::makeEmpty<connection_types::NoMessage>() : value.front()->toType();
}

TokenData::ConstPtr VectorMessage::nestedValue(std::size_t i) const
{
    return  value.at(i);
}
std::size_t VectorMessage::nestedValueCount() const
{
    return value.size();
}
void VectorMessage::addNestedValue(const TokenData::ConstPtr &msg)
{
    value.push_back(msg);
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
    rhs.value = node["values"].as<std::vector<TokenDataConstPtr>>();
    return true;
}
}
