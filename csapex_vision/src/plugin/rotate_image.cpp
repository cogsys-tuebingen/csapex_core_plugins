/// HEADER
#include "rotate_image.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::RotateImage, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


RotateImage::RotateImage()
{
}

void RotateImage::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("angle", -M_PI, M_PI, 0.0, 0.001));
}

void RotateImage::setup()
{
    in_  = modifier_->addInput<CvMatMessage>("image");
    out_ = modifier_->addOutput<CvMatMessage>("rotated image");
}

void RotateImage::process()
{
    CvMatMessage::Ptr src = in_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr dst(new CvMatMessage(src->getEncoding()));

    double angle = readParameter<double>("angle");
    int dim = std::max(src->value.cols, src->value.rows);

    cv::Point2d pt(dim/2.0, dim/2.0);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle / M_PI * 180.0, 1.0);

    cv::warpAffine(src->value, dst->value, r, cv::Size(dim, dim), cv::INTER_NEAREST);

    out_->publish(dst);
}

