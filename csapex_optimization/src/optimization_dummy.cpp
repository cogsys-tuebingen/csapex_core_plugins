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

    ainfo << "params are a: " << a << ", b: " << b << ", c: " << c << std::endl;

    double fitness = std::pow(a-b, 2) + c;
    ainfo << "dummy optimizer: fitness = " << fitness;

    out_->publish(fitness);
}

