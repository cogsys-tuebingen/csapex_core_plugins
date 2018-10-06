/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/token.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/signal/event.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex::connection_types;

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN SignalDelayMeasurement : public Node
{
public:
    void setup(csapex::NodeModifier& modifier) override
    {
        start_ = std::chrono::system_clock::now();

        time_out_ = modifier.addEvent<double>("time_delay_ms");

        modifier.addSlot("start", [this]() { start_ = std::chrono::system_clock::now(); });
        modifier.addSlot("stop", [this]() {
            end_ = std::chrono::system_clock::now();

            std::chrono::system_clock::duration delta = end_ - start_;

            double time_diff = delta.count() * 1e-6;
            ainfo << "Measured delay: " << time_diff << "ms" << std::endl;

            GenericValueMessage<double>::Ptr msg_out(new GenericValueMessage<double>);
            msg_out->value = time_diff;
            time_out_->triggerWith(std::make_shared<Token>(msg_out));
        });
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start_;
    std::chrono::time_point<std::chrono::system_clock> end_;
    Event* time_out_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::SignalDelayMeasurement, csapex::Node)
