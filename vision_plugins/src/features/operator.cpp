#include "operator.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
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
//    std::map<std::string, int> depths = boost::assign::map_list_of
//            ("default", -1)
//            ("CV_16S", CV_16S)
//            ("CV_32F", CV_32F)
//            ("CV_64F", CV_64F);

//    addParameter(param::ParameterFactory::declareParameterSet("ddepth", depths),
//                 boost::bind(&Operator::update, this));
    addParameter(param::ParameterFactory::declareRange("kernel", 1, 31, ksize_, 2),
                 boost::bind(&Operator::update, this));
    addParameter(param::ParameterFactory::declareRange("scale", 0.1, 10.0, scale_, 0.1),
                 boost::bind(&Operator::update, this));
    addParameter(param::ParameterFactory::declareRange("delta", 0.0, 10.0, delta_, 0.1),
                 boost::bind(&Operator::update, this));
}

void Operator::setup()
{
    CornerLineDetection::setup();
    update();
}

void Operator::update()
{
//    ddepth_ = param<int>("ddepth");
    ksize_  = param<int>("kernel");
    scale_  = param<double>("scale");
    delta_  = param<double>("delta");
}
