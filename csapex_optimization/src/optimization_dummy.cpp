/// HEADER
#include "optimization_dummy.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::OptimizationDummy, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


OptimizationDummy::OptimizationDummy()
    : evaluate_(false)
{
}

void OptimizationDummy::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("a", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("b", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("c", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("d", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("e", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("f", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("g", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("h", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("i", -10.0, 10.0, 0.0, 0.1));
    parameters.addParameter(param::ParameterFactory::declareRange("j", -10.0, 10.0, 0.0, 0.1));
}

void OptimizationDummy::setup(NodeModifier& node_modifier)
{
    in_  = node_modifier.addSlot("Evaluate", std::bind(&OptimizationDummy::start, this));
    out_ = node_modifier.addOutput<double>("Fitness");
}

void OptimizationDummy::start()
{
    evaluate_ = true;
}

bool OptimizationDummy::canTick()
{
    return evaluate_;
}

void OptimizationDummy::tick()
{
    evaluate_ = false;

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


    double fitness = std::pow(0.0 - a, 2)
            + std::pow(1.0 - b, 2)
            + std::pow(2.0 - c, 2)
            + std::pow(3.0 - d, 2)
            + std::pow(4.0 - e, 2)
            + std::pow(5.0 - f, 2)
            + std::pow(6.0 - g, 2)
            + std::pow(7.0 - h, 2)
            + std::pow(8.0 - i, 2)
            + std::pow(9.0 - j, 2)
            ;

    msg::publish(out_, fitness);
}

void OptimizationDummy::process()
{

}
