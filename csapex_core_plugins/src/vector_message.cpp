/// HEADER
#include <csapex_core_plugins/vector_message.h>

using namespace csapex;
using namespace connection_types;

GenericVectorMessage::GenericVectorMessage(ConnectionType::Ptr impl, const std::string& frame_id)
    : Message (impl->name(), frame_id), impl(impl)
{
}

ConnectionType::Ptr GenericVectorMessage::clone() {
    Ptr new_msg(new GenericVectorMessage(impl->clone(), frame_id));
    return new_msg;
}

ConnectionType::Ptr GenericVectorMessage::toType() {
    Ptr new_msg(new GenericVectorMessage(AnyMessage::make(), frame_id));
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

//// OLD



VectorMessage::VectorMessage(const std::string& frame_id)
    : Message ("std::vector<?>", frame_id)
{
    type_ = AnyMessage::make();
}
VectorMessage::VectorMessage(ConnectionType::Ptr type, const std::string& frame_id)
    : Message (std::string("std::vector<") + type->rawName()  + "::Ptr>", frame_id)
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
