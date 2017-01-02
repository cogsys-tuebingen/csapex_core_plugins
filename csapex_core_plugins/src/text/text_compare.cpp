
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/signal/event.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class TextCompare : public Node
{
public:
    TextCompare()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_a_ = modifier.addInput<std::string>("A");
        in_b_ = modifier.addInput<std::string>("B");
        e_ = modifier.addEvent<GenericValueMessage<std::string>>("A=B");
        ne_ = modifier.addEvent<GenericValueMessage<std::string>>("A!=B");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process() override
    {
        std::string A = msg::getValue<std::string>(in_a_);
        std::string B = msg::getValue<std::string>(in_b_);

        if(A == B) {
            e_->trigger();
        } else {
            ne_->trigger();
        }
    }

private:
    Input* in_a_;
    Input* in_b_;
    Event* e_;
    Event* ne_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::TextCompare, csapex::Node)

