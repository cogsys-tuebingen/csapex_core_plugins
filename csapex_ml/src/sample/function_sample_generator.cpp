
/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ml/features_message.h>

/// SYSTEM
#include <random>

using namespace csapex;
using namespace csapex::connection_types;

namespace csapex
{


class FunctionSampleGenerator : public Node
{
public:
    FunctionSampleGenerator()
    {
    }

    void setup(csapex::NodeModifier& modifier) override
    {
        out_ = modifier.addOutput<FeaturesMessage>("Sample");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
        params.addParameter(param::ParameterFactory::declareValue("seed", 0), [this](param::Parameter* p) {
            twister.seed(p->as<int>());
        });
        params.addParameter(param::ParameterFactory::declareRange("outlier_prob", 0.0, 1.0, 0.1, 0.001), outlier_prob_);

    }

    void process() override
    {
        // TODO: make function configurable
        double rval_rel = twister() / double(std::numeric_limits<uint_fast64_t>::max());

        double x = rval_rel * 2.0 * M_PI;
        double y_t = std::cos(x);
        double y = y_t;

        FeaturesMessage::Ptr feature = std::make_shared<FeaturesMessage>();

        double error_p = randnum() < outlier_prob_;
        if(error_p) {
            double error = 0.1 + randnum() * 2.0;
            double error_sign = randnum() >= 0.5 ? 1 : -1;
            y += error * error_sign;
        }

        double sigma_noise = 0.1;
        double noise = (randnum() - 0.5) * sigma_noise;
        y += noise;

        if(std::abs(y - y_t) < sigma_noise) {
            feature->classification = 1;
        } else {
            feature->classification = 0;
        }

        feature->value.push_back(x);
        feature->value.push_back(y);

        msg::publish(out_, feature);
    }

    double randnum()
    {
        return twister() / double(std::numeric_limits<uint_fast64_t>::max());
    }

private:
    Output* out_;

    double outlier_prob_;

    std::mt19937_64 twister;

};

} // csapex


CSAPEX_REGISTER_CLASS(csapex::FunctionSampleGenerator, csapex::Node)

