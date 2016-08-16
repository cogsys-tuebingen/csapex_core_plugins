/// PROJECT
#include <csapex/model/node.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/generic_value_message.hpp>

namespace csapex
{

class CSAPEX_EXPORT_PLUGIN Statistics : public Node
{
public:
    Statistics()
    {
        reset();
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        in_ = node_modifier.addMultiInput<double, int>("number");

        out_mean_ = node_modifier.addOutput<double>("mean");
        out_std_dev_ = node_modifier.addOutput<double>("std dev");
    }

    void setupParameters(Parameterizable& parameters)
    {
        parameters.addParameter(param::ParameterFactory::declareTrigger("reset"),
                                [this](param::Parameter*) {
            reset();
        });
    }

    void process()
    {
        double x = 0.0;

        if(msg::isMessage<connection_types::GenericValueMessage<double>>(in_)) {
            x = msg::getValue<double>(in_);

        } else if(msg::isMessage<connection_types::GenericValueMessage<int>>(in_)) {
            x = msg::getValue<int>(in_);

        } else {
            throw std::runtime_error("invalid input type");
        }

        double prev_mean = m;
        ++n;
        m = m + (x-m)/n;
        S = S + (x-m)*(x-prev_mean);

        msg::publish(out_mean_, m);
        msg::publish(out_std_dev_, S);
    }

    void reset()
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

}

CSAPEX_REGISTER_CLASS(csapex::Statistics, csapex::Node)
