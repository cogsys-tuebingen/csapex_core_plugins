/// HEADER
#include <csapex_core_plugins/vector_message.h>

using namespace csapex;
using namespace connection_types;

GenericVectorMessage::GenericVectorMessage(ConnectionType::Ptr impl)
    : Message (impl->name()), impl(impl)
{
}

ConnectionType::Ptr GenericVectorMessage::clone() {
    Ptr new_msg(new GenericVectorMessage(impl->clone()));
    return new_msg;
}

ConnectionType::Ptr GenericVectorMessage::toType() {
    Ptr new_msg(new GenericVectorMessage(AnyMessage::make()));
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

void GenericVectorMessage::writeYaml(YAML::Emitter& yaml) const {
    impl->writeYaml(yaml);
}
void GenericVectorMessage::readYaml(const YAML::Node& node) {
    impl->readYaml(node);
}


//// OLD



VectorMessage::VectorMessage()
    : Message ("std::vector<?>")
{
    type_ = AnyMessage::make();
}
VectorMessage::VectorMessage(ConnectionType::Ptr type)
    : Message (std::string("std::vector<") + type->rawName()  + "::Ptr>")
{
    type_ = type;
}

ConnectionType::Ptr VectorMessage::getSubType() const
{
    return type_;
}

VectorMessage::Ptr VectorMessage::make(){
    Ptr new_msg(new VectorMessage);
    return new_msg;
}

ConnectionType::Ptr VectorMessage::clone() {
    Ptr new_msg(new VectorMessage);
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

void VectorMessage::writeYaml(YAML::Emitter& yaml) const {
    yaml << YAML::Key << "vector" << YAML::Value << YAML::Flow;
    yaml << YAML::BeginSeq;
    for(unsigned i = 0; i < value.size(); ++i) {
        yaml << YAML::BeginMap;
        value[i]->writeYaml(yaml);
        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;
}
void VectorMessage::readYaml(const YAML::Node& node) {
    throw std::runtime_error("deserialization of vectors not implemented");
}
