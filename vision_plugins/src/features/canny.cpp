#include "canny.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::Canny, csapex::Node)

Canny::Canny() :
    threshold_1_(3),
    threshold_2_(0.0),
    aperture_size_(255.0),
    L2_gradient_(false)
{
}

void Canny::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();

    if(in->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono));

    cv::Mat edges;

    cv::Canny(in->value, edges, threshold_1_, threshold_2_, aperture_size_, L2_gradient_);

    out->value.create(in->value.size(), in->value.type());
    out->value.setTo(cv::Scalar::all(0));
    in->value.copyTo(out->value,edges);

    output_->publish(out);
}

void Canny::setup()
{
    CornerLineDetection::setup();
    update();
}

void Canny::setupParameters()
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

void Canny::update()
{
    threshold_1_   = readParameter<double>("threshold 1");
    threshold_2_   = readParameter<double>("threshold 2");
    aperture_size_ = readParameter<int>("aperture");
    L2_gradient_   = readParameter<bool>("L2 gradient");
}
