
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
#include <random>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class ParameterVectorAddNoise : public Node
{
public:
    ParameterVectorAddNoise()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<GenericVectorMessage,double>("Input");
        out_ = modifier.addOutput<GenericVectorMessage, double>("Output");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::factory::declareRange("rel_standard_deviation",0.0,1.0,0.1,0.01),
                            rel_std_dev_);
    }

    void process() override
    {
        auto params = msg::getMessage<GenericVectorMessage, double>(in_);
        std::shared_ptr<std::vector<double>> out;
        out->resize(params->size());
        auto it = out->begin();
        for(auto val : *params){
            double std_dev = val * rel_std_dev_;
            std::normal_distribution<double> dist(val,std_dev);
            *it = (double) dist(generator_);
        }

        msg::publish<GenericVectorMessage, double>(out_, out);
    }

private:
    Input* in_;
    Output* out_;
    double rel_std_dev_;
    std::default_random_engine generator_;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::ParameterVectorAddNoise, csapex::Node)

