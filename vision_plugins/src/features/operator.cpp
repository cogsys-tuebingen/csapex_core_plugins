#include "operator.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

Operator::Operator() :
    ksize_(1),
    scale_(1.0),
    delta_(0.0)
{
}

void Operator::setup(NodeModifier& node_modifier)
{
    CornerLineDetection::setup(node_modifier);
    update();
}

void Operator::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("kernel", 1, 31, ksize_, 2),
                 std::bind(&Operator::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("scale", -10.0, 10.0, scale_, 0.01),
                 std::bind(&Operator::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("delta", -100.0, 100.0, delta_, 0.01),
                 std::bind(&Operator::update, this));
}

void Operator::update()
{
    ksize_  = readParameter<int>("kernel");
    scale_  = readParameter<double>("scale");
    delta_  = readParameter<double>("delta");
}
