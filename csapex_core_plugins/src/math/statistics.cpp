/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN Statistics : public Node
{
public:
    Statistics()
    {
        reset();
    }

    void setup(csapex::NodeModifier& node_modifier) override
    {
        in_ = node_modifier.addMultiInput<double, int>("number");

        out_mean_ = node_modifier.addOutput<double>("mean");
        out_std_dev_ = node_modifier.addOutput<double>("std dev");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addParameter(param::factory::declareTrigger("reset"), [this](param::Parameter*) { reset(); });

        parameters.addParameter(param::factory::declareOutputText("mean"));
        parameters.addParameter(param::factory::declareOutputText("std dev"));
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

        double prev_mean = m;
        ++n;
        m = m + (x - m) / n;
        S = ((n - 1) * S + (x - m) * (x - prev_mean)) / ((double)n);

        double std = sqrt(S);
        setParameter("mean", "mean : " + std::to_string(m));
        setParameter("std dev", "std dev: " + std::to_string(S));

        msg::publish(out_mean_, m);
        msg::publish(out_std_dev_, std);
    }

    void reset() override
    {
        S = 0;
        m = 0;
        n = 0;
    }

private:
    Input* in_;

    Output* out_mean_;
    Output* out_std_dev_;

    double m;
    double S;
    int n;
};

}  // namespace csapex

CSAPEX_REGISTER_CLASS(csapex::Statistics, csapex::Node)
