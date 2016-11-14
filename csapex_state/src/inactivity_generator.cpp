
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/signal/event.h>
#include <csapex/msg/any_message.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/token.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
namespace state
{

class InactivityGenerator : public Node
{
public:
    InactivityGenerator()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        out_ = modifier.addEvent("Deactivation");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareTrigger("trigger"), [this](param::Parameter* p) {
            auto new_val = std::make_shared<connection_types::AnyMessage>();
            TokenPtr token = std::make_shared<Token>(new_val);
            token->setActivityModifier(ActivityModifier::DEACTIVATE);
            out_->triggerWith(token);
        });
    }

    void activation() override
    {
    }

    void deactivation() override
    {
    }

    void process() override
    {
    }

private:
    Event* out_;
};

}
}

CSAPEX_REGISTER_CLASS(csapex::state::InactivityGenerator, csapex::Node)

