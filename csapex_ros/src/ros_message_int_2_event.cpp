/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/event.h>
/// SYSTEM
#include <string>
using namespace csapex::connection_types;
namespace csapex
{
class RosMessageInt2Event : public Node
{
public:
    RosMessageInt2Event():
        last_val_(0)
    {}

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addInput<GenericValueMessage<int>>("input");
        changed_ = node_modifier.addEvent("changed");


    }

    void setupParameters(csapex::Parameterizable& parameters) override
    {

    }




    void process()
    {

        GenericValueMessage<int>::ConstPtr in = msg::getMessage<connection_types::GenericValueMessage<int>>(in_);
        if( in->value != last_val_)
        {
            changed_->trigger();
        }
        last_val_ = in->value;
    }



private:

    Input* in_;
    Event* changed_;
    int last_val_;



};

}
CSAPEX_REGISTER_CLASS(csapex::RosMessageInt2Event, csapex::Node)





