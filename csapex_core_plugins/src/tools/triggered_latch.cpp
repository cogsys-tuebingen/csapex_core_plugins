
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/any_message.h>
#include <csapex/model/token.h>
#include <csapex/msg/marker_message.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class TriggeredLatch : public Node
{
public:
    TriggeredLatch()
        : publish_once_(false)
    {
    }
    void setup(csapex::NodeModifier& modifier) override
    {
        modifier.addTypedSlot<AnyMessage>("Message", [this](const TokenPtr& token) {
            auto marker = std::dynamic_pointer_cast<MarkerMessage const>(token->getTokenData());
            if(marker != nullptr ^ ignore_markers_) {
                data_ = token->getTokenData();
                yield();
            }
        });
        modifier.addSlot("reset", [this](){
            data_.reset();
            publish_once_ = false;
        });
        modifier.addSlot("publish", [this](){
            publish_once_ = true;
            yield();
        });
        out_ = modifier.addOutput<AnyMessage>("Throttled");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareBool("ignore_marker_tokens", true), ignore_markers_);
    }

    bool canProcess() const override
    {
        return data_ != nullptr && publish_once_;
    }

    void process() override
    {
        msg::publish(out_, data_);
        publish_once_ = false;
    }

private:
    Output* out_;

    TokenDataConstPtr data_;

    bool publish_once_;
    bool ignore_markers_;
};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::TriggeredLatch, csapex::Node)

