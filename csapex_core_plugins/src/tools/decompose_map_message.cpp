
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_core_plugins/composite_message.h>
#include <csapex/msg/output.h>
#include <csapex/serialization/serialization.h>
#include <csapex/msg/any_message.h>
#include <csapex/utility/yaml_io.hpp>
#include <csapex_core_plugins/map_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class CSAPEX_EXPORT_PLUGIN DecomposeMapMessage : public Node
{
public:
    DecomposeMapMessage()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<MapMessage>("Map");

    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        MapMessage::ConstPtr  message = msg::getMessage<MapMessage>(in_);

        std::size_t count = message->value.size();

        updateOutputs(count);

        std::size_t i = 0;
        for(KeyValueMessage part : message->value) {
            Output* o = outputs_[i++];

            std::string label = part.value.first + ": " + part.value.second->typeName();
            msg::setLabel(o, label);
            o->setType(part.value.second->toType());

            msg::publish(o, part.value.second);
        }
    }

private:
    void updateOutputs(std::size_t count)
    {
        if(count != outputs_.size()) {
            for(std::size_t i = outputs_.size(); i < count; ++i) {
                outputs_.push_back(node_modifier_->addOutput<AnyMessage>("Part"));
            }
        }
    }

private:
    Input* in_;
    std::vector<Output*> outputs_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::DecomposeMapMessage, csapex::Node)

