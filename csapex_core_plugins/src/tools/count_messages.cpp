/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/token_data.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/msg/message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN CountMessages : public Node
{
public:
    CountMessages() : input_(nullptr), output_(nullptr), reset_(nullptr), count_(0)
    {
    }

    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        input_ = node_modifier.addInput<connection_types::AnyMessage>("Anything");
        output_ = node_modifier.addOutput<int>("Count");
        reset_ = node_modifier.addSlot("Reset", std::bind(&CountMessages::reset, this));
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareTrigger("reset"), std::bind(&CountMessages::reset, this));
    }

    virtual void process() override
    {
        ++count_;
        msg::publish(output_, count_);
    }

private:
    Input* input_;
    Output* output_;
    Slot* reset_;

    int count_;

    void reset() override
    {
        count_ = 0;
    }
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::CountMessages, csapex::Node)
