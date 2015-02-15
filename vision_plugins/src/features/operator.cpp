#include "operator.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <boost/assign.hpp>
#include <utils_cv/normalization.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

Operator::Operator() :
    ksize_(1),
    scale_(1.0),
    delta_(0.0)
{
}

void Operator::setup()
{
    CornerLineDetection::setup();
    update();
}

void Operator::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("kernel", 1, 31, ksize_, 2),
                 std::bind(&Operator::update, this));
    addParameter(param::ParameterFactory::declareRange("scale", -10.0, 10.0, scale_, 0.01),
                 std::bind(&Operator::update, this));
    addParameter(param::ParameterFactory::declareRange("delta", -100.0, 100.0, delta_, 0.01),
                 std::bind(&Operator::update, this));
}

void Operator::update()
{
    ksize_  = readParameter<int>("kernel");
    scale_  = readParameter<double>("scale");
    delta_  = readParameter<double>("delta");
}
