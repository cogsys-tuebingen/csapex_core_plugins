/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_core_plugins/composite_message.h>
#include <csapex/msg/output.h>
#include <csapex/serialization/serialization.h>


using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{

class Decomposer : public Node
{
    friend class DecomposerSerializer;

public:
    Decomposer()
    {}

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_ = node_modifier.addInput<CompositeMessage>("Composed");
    }

    void setupParameters(Parameterizable& parameters)
    {
    }

    void process()
    {
        CompositeMessage::ConstPtr message = msg::getMessage<CompositeMessage>(input_);

        std::size_t n = message->value.size();

        updateOutputs(n);

        std::size_t i = 0;
        for(ConnectionTypeConstPtr part : message->value) {
            Output* o = outputs_[i++];

            msg::setLabel(o, part->typeName());
            o->setType(part->toType());

            msg::publish(o, part);
        }
    }

private:
    void updateOutputs(std::size_t count)
    {
        if(count != outputs_.size()) {
            for(std::size_t i = outputs_.size(); i < count; ++i) {
                outputs_.push_back(modifier_->addOutput<AnyMessage>("Part"));
            }
        }
    }

private:
    Input* input_;

    std::vector<Output*> outputs_;
};

}

CSAPEX_REGISTER_CLASS(csapex::Decomposer, csapex::Node)


namespace csapex
{
class DecomposerSerializer
{
public:
    static void serialize(const Decomposer& decomposer, YAML::Node& doc)
    {
        doc["channel_count"] = decomposer.outputs_.size();
    }

    static void deserialize(Decomposer& decomposer, const YAML::Node& doc)
    {
        int c = 0;
        if(doc["channel_count"].IsDefined()) {
            c = doc["channel_count"].as<int>();
        }

        decomposer.updateOutputs(c);
    }
};
}

CSAPEX_REGISTER_SERIALIZER(csapex::Decomposer, DecomposerSerializer)
