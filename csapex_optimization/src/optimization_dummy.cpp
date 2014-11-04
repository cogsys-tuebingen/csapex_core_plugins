/// HEADER
#include "optimization_dummy.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::OptimizationDummy, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


OptimizationDummy::OptimizationDummy()
{
}

void OptimizationDummy::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("a", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("b", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("c", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("d", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("e", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("f", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("g", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("h", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("i", -10.0, 10.0, 0.0, 0.1));
    addParameter(param::ParameterFactory::declareRange("j", -10.0, 10.0, 0.0, 0.1));
}

void OptimizationDummy::setup()
{
    in_  = modifier_->addInput<AnyMessage>("Trigger");
    out_ = modifier_->addOutput<double>("Fitness");
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

    ainfo << "params are a: " << a << ", b: " << b << ", c: " << c << std::endl;

//    double fitness = a*a + b*b + c*c;
    double fitness = (a - b) + c * d + e * (f - g) + i + j;
    ainfo << "dummy optimizer: fitness = " << fitness << std::endl;

    out_->publish(fitness);
}

