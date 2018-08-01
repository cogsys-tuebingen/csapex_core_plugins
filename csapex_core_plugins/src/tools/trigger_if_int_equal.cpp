
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


class TiggerIfIntEqual : public Node
{
public:
    TiggerIfIntEqual()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<int>("Input");
        true_  = modifier.addEvent("true");
        false_ = modifier.addEvent("false");
    }
    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareValue("value",0),
                            value_);
    }

    void process() override
    {
        int value = msg::getValue<int>(in_);
        if(value_ == value){
            true_->trigger();
        }
        else{
            false_->trigger();
        }

    }

private:
    Input* in_;
    Event* true_;
    Event* false_;
    int value_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::TiggerIfIntEqual, csapex::Node)

