
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/any_message.h>
#include <csapex_core_plugins/key_value_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class CreateKeyValueMessage : public Node
{
public:
    CreateKeyValueMessage()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<AnyMessage>("Input");
        out_ = modifier.addOutput<KeyValueMessage>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareText(
                                "key",
                               param::ParameterDescription("key of the KeyValueMessage (name of the message)"),
                                "something"),
                            key_);
    }


    void process() override
    {
        TokenData::ConstPtr msg = msg::getMessage<TokenData>(in_);

        KeyValueMessage::ConstPtr key_value(new KeyValueMessage(key_, msg));

        msg::publish(out_, key_value);

    }

private:
    Input* in_;
    Output* out_;

    std::string key_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::CreateKeyValueMessage, csapex::Node)

