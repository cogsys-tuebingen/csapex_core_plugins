
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/variadic_io.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_core_plugins/map_message.h>
#include <csapex/param/parameter_factory.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class CSAPEX_EXPORT_PLUGIN CreateMapMessage : public Node, public VariadicInputs
{
public:
    CreateMapMessage()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        setupVariadic(modifier);

        out_ = modifier.addOutput<MapMessage>("map");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        setupVariadicParameters(params);
    }

    void process() override
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

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override
    {
        std::size_t i = getVariadicInputCount();
        param::Parameter::Ptr key_param = param::ParameterFactory::declareText("key_" +std::to_string(i),
                                                                               "name");

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
        addTemporaryParameter(key_param, cb);

        return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::AnyMessage>(), label.empty() ? "Value" : label, getVariadicInputCount() == 0 ? false : true);
    }

private:
    Input* in_;
    Output* out_;

    std::vector<param::Parameter::Ptr> params_keys_;
    std::vector<std::string> keys_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::CreateMapMessage, csapex::Node)

