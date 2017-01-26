#include "create_map_message.h"
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_core_plugins/map_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/serialization/serialization.h>

using namespace csapex;
using namespace csapex::connection_types;


CSAPEX_REGISTER_CLASS(csapex::CreateMapMessage, csapex::Node)

CreateMapMessage::CreateMapMessage()
{
}

void CreateMapMessage::setup(csapex::NodeModifier& modifier)
{
    setupVariadic(modifier);

    out_ = modifier.addOutput<MapMessage>("map");
}

void CreateMapMessage::setupParameters(csapex::Parameterizable& params)
{
    setupVariadicParameters(params);
}

void CreateMapMessage::process()
{
    MapMessage::Ptr result(new MapMessage);

    std::vector<InputPtr> inputs = node_modifier_->getMessageInputs();
    for(std::size_t i = 0 ; i < inputs.size() ; i++) {
        Input *in = inputs[i].get();
        if(msg::hasMessage(in)) {
            TokenData::ConstPtr m =  msg::getMessage(in);

            if(keys_.size() > i){
                KeyValueMessage tmp(keys_[i],m);
                result->value.emplace_back(tmp);
            }
        }
    }

    msg::publish(out_, result);
}

csapex::Input* CreateMapMessage::createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional)
{
    std::size_t i = getVariadicInputCount();
    std::string param_name = std::string("key_") + std::to_string(i);

    param::Parameter::Ptr key_param;
    if(hasParameter(param_name)) {
        key_param = getParameter(param_name);
    } else  {
        key_param = param::ParameterFactory::declareText(param_name, "name");
    }

    keys_.push_back("");

    std::function<void(param::Parameter* p)> cb = [this,i](param::Parameter* p){
        if(p->is<std::string>()){
            keys_.at(i)=(p->as<std::string>());
        }
    };
    if(!key_param) {
        throw std::runtime_error("Could not create temporary parameter!");
    }

    params_keys_.emplace_back(key_param);
    addPersistentParameter(key_param);
    addParameterCallback(key_param, cb);

    return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
}



