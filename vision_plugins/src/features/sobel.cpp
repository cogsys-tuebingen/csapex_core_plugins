#include "sobel.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/normalization.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Sobel, csapex::Node)

Sobel::Sobel() :
    dx_(1),
    dy_(1)
{
}


void Sobel::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp));
    int depth = in->value.type() & 7;
    cv::Sobel(in->value, out->value, depth, dx_, dy_, ksize_, scale_,delta_);
    output_->publish(out);
}

void Sobel::setupParameters()
{
    Operator::setupParameters();
    addParameter(param::ParameterFactory::declareRange("dx", 0, 5, dx_, 1),
                 boost::bind(&Sobel::update, this));
    addParameter(param::ParameterFactory::declareRange("dy", 0, 5, dy_, 1),
                 boost::bind(&Sobel::update, this));
}

void  Sobel::update()
{
    Operator::update();
    dx_     = readParameter<int>("dx");
    dy_     = readParameter<int>("dy");
}
