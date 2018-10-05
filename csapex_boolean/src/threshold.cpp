
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{
class Threshold : public Node
{
public:
    Threshold()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addMultiInput<double, int>("number");
        out_ = modifier.addOutput<bool>("Predicate");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareValue("threshold", 0.0), threshold_);
    }

    void process() override
    {
        double x = 0.0;

        if (msg::isMessage<connection_types::GenericValueMessage<double>>(in_)) {
            x = msg::getValue<double>(in_);

        } else if (msg::isMessage<connection_types::GenericValueMessage<int>>(in_)) {
            x = msg::getValue<int>(in_);

        } else {
            throw std::runtime_error("invalid input type");
        }

        msg::publish(out_, x > threshold_);
    }

private:
    Input* in_;
    Output* out_;

    double threshold_;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Threshold, csapex::Node)
