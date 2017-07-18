#ifndef ROSMESSAGEBOOL2EVENT_H
#define ROSMESSAGEBOOL2EVENT_H
/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>
#include <csapex/signal/event.h>
namespace csapex {
class RosMessageBool2Event: public Node
{
public:
    RosMessageBool2Event();

    void setup(csapex::NodeModifier& node_modifier) override;

    void setupParameters(csapex::Parameterizable& parameters) override;

    void process();

private:
    Input* in_;
    Event* event_true_;
    Event* event_false_;
    Event* event_changed_;

    bool last_;
};
}
#endif // ROSMESSAGEBOOL2EVENT_H
