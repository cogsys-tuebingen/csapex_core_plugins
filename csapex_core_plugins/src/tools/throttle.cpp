
/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_state.h>
#include <csapex/model/throttled_node.h>
#include <csapex/model/token.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class Throttle : public ThrottledNode
{
public:
    Throttle()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        modifier.addSlot<AnyMessage>("Message", [this](const TokenPtr& token) {
            data_ = token->getTokenData();
            yield();
        });
        out_ = modifier.addOutput<AnyMessage>("Throttled");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        ThrottledNode::setupParameter(params, "frequency");
    }

    bool canProcess() const override
    {
        return data_ != nullptr;
    }

    void process() override
    {
        msg::publish(out_, data_);
        data_.reset();
    }

private:
    Output* out_;

    TokenDataConstPtr data_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Throttle, csapex::Node)
