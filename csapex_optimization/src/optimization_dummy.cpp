/// HEADER
#include "optimization_dummy.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::OptimizationDummy, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

OptimizationDummy::OptimizationDummy()
{
}

void OptimizationDummy::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange<double>("a", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("b", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("c", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("d", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("e", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("f", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("g", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("h", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("i", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(csapex::param::factory::declareRange<double>("j", -10.0, 10.0, 0.0, 0.1));

    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/a", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/b", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/c", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/d", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/e", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/f", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/g", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/h", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/i", false));
    parameters.addParameter(csapex::param::factory::declareBool("~calculate_grad/j", false));
}

void OptimizationDummy::setup(NodeModifier& node_modifier)
{
    in_ = node_modifier.addInput<double>("argument");
    out_ = node_modifier.addOutput<double>("Fitness");
    out_grad_ = node_modifier.addOutput<GenericVectorMessage, double>("gradient");
}

void OptimizationDummy::process()
{
    double a = readParameter<double>("a");
    double b = readParameter<double>("b");
    double c = readParameter<double>("c");
    double d = readParameter<double>("d");
    double e = readParameter<double>("e");
    double f = readParameter<double>("f");
    double g = readParameter<double>("g");
    double h = readParameter<double>("h");
    double i = readParameter<double>("i");
    double j = readParameter<double>("j");

    double x = msg::getValue<double>(in_);

    double fitness = std::pow(a - 4, 2) + std::pow(x - 5.0 - b, 2) + std::pow(x - 2.0 - c, 2) + std::pow(x - 3.0 - d, 2) + std::pow(x - 4.0 - e, 2) + std::pow(x - 5.0 - f, 2) +
                     std::pow(x - 6.0 - g, 2) + std::pow(x - 7.0 - h, 2) + std::pow(x - 8.0 - i, 2) + std::pow(x - 9.0 - j, 2);

    msg::publish(out_, fitness);

    // d fitness /( d p), where p = { a, b, c, d, e, f, g, h ,i ,j};
    if (msg::isConnected(out_grad_)) {
        std::shared_ptr<std::vector<double>> grad(new std::vector<double>);
        if (readParameter<bool>("~calculate_grad/a")) {
            grad->push_back(2.0 * (a - 4));
        }
        if (readParameter<bool>("~calculate_grad/b")) {
            grad->push_back(-2.0 * (x - 5.0 - b));
        }
        if (readParameter<bool>("~calculate_grad/c")) {
            grad->push_back(-2.0 * (x - 2.0 - c));
        }
        if (readParameter<bool>("~calculate_grad/d")) {
            grad->push_back(-2.0 * (x - 3.0 - d));
        }
        if (readParameter<bool>("~calculate_grad/e")) {
            grad->push_back(-2.0 * (x - 4.0 - e));
        }
        if (readParameter<bool>("~calculate_grad/f")) {
            grad->push_back(-2.0 * (x - 5.0 - f));
        }
        if (readParameter<bool>("~calculate_grad/g")) {
            grad->push_back(-2.0 * (x - 6.0 - g));
        }
        if (readParameter<bool>("~calculate_grad/h")) {
            grad->push_back(-2.0 * (x - 7.0 - h));
        }
        if (readParameter<bool>("~calculate_grad/i")) {
            grad->push_back(-2.0 * (x - 8.0 - i));
        }
        if (readParameter<bool>("~calculate_grad/j")) {
            grad->push_back(-2.0 * (x - 9.0 - j));
        }
        msg::publish<GenericVectorMessage, double>(out_grad_, grad);
    }
}
