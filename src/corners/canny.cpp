#include "canny.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::Canny, csapex::Node)

Canny::Canny() :
    threshold_1_(3),
    threshold_2_(0.0),
    aperture_size_(255.0),
    L2_gradient_(false)
{
    addParameter(param::ParameterFactory::declareRange("aperture", 3, 7, 3, 2),
                 boost::bind(&Canny::update, this));
    addParameter(param::ParameterFactory::declareRange("threshold 1", 0.0, 500.0, 0.0, 1.0),
                 boost::bind(&Canny::update, this));
    addParameter(param::ParameterFactory::declareRange("threshold 2", 0.0, 500.0, 255.0, 1.0),
                 boost::bind(&Canny::update, this));
    addParameter(param::ParameterFactory::declareBool("L2 gradient", false),
                 boost::bind(&Canny::update, this));
}

void Canny::allConnectorsArrived()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    CvMatMessage::Ptr out(new connection_types::CvMatMessage);

    cv::Mat tmp;
    cv::Mat edges;
    if(in->encoding.size() != 1 ) {
        cv::cvtColor(in->value, tmp, CV_BGR2GRAY);
    } else {
        tmp = in->value;
    }

    cv::Canny(tmp, edges, threshold_1_, threshold_2_, aperture_size_, L2_gradient_);
    out->value.create(in->value.size(), in->value.type());
    out->value.setTo(cv::Scalar::all(0));
    in->value.copyTo(out->value,edges);
    out->encoding = in->encoding;
    output_->publish(out);
}

void Canny::setup()
{
    CornerDetection::setup();
    update();
}


void Canny::update()
{
    threshold_1_   = param<double>("threshold 1");
    threshold_2_   = param<double>("threshold 2");
    aperture_size_ = param<int>("aperture");
    L2_gradient_   = param<bool>("L2 gradient");
}
