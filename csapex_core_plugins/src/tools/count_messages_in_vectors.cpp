/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/message.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/model/node.h>
#include <csapex/model/token_data.h>
#include <csapex/signal/slot.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN CountMessagesInVectors : public Node
{
public:
    CountMessagesInVectors()
        : input_(nullptr), output_(nullptr), reset_(nullptr), count_(0)
    {
    }


    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<connection_types::GenericVectorMessage>("Anything");
        output_ = node_modifier.addOutput<std::size_t>("Count");
        reset_ = node_modifier.addSlot("Reset",
                                        std::bind(&CountMessagesInVectors::reset, this));
    }

    virtual void setupParameters(Parameterizable &parameters) override
    {
        parameters.addParameter(param::factory::declareTrigger("reset"),
                                std::bind(&CountMessagesInVectors::reset, this));
    }

    virtual void process() override
    {
        connection_types::GenericVectorMessage::ConstPtr vector =
                msg::getMessage<connection_types::GenericVectorMessage>(input_);
        count_ += vector->nestedValueCount();

        msg::publish(output_, count_);
    }

private:
    Input* input_;
    Output* output_;
    Slot*   reset_;

    std::size_t count_;

    void reset()
    {
        count_ = 0;
    }

};

}

CSAPEX_REGISTER_CLASS(csapex::CountMessagesInVectors, csapex::Node)
