/// HEADER
#include "threshold_noise_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <utils_cv/noise_filter.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::ThresholdNoiseFilter, csapex::Node)

ThresholdNoiseFilter::ThresholdNoiseFilter()
{
}


void ThresholdNoiseFilter::process()
{
    CvMatMessage::ConstPtr input = msg::getMessage<CvMatMessage>(input_);
    CvMatMessage::ConstPtr threshold = msg::getMessage<CvMatMessage>(threshold_);
    CvMatMessage::Ptr output(new CvMatMessage(input->getEncoding(), input->stamp_micro_seconds));

    if(!threshold->hasChannels(1, CV_8U)) {
        throw std::runtime_error("Threshold needs to be one channel grayscale!");
    }

    bool  interpolate = readParameter<bool>("interpolate");
    uchar thresh = readParameter<int>("threshold");
    int   type   = input->value.type();
    switch(type) {
    case CV_8UC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<uchar,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<uchar,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    case CV_8SC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<char,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<char,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    case CV_16UC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<ushort,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<ushort,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    case CV_16SC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<short,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<short,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    case CV_32SC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<int,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<int,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    case CV_32FC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<float,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<float,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    case CV_64FC1:
        if(interpolate)
            utils_cv::ThresholdNoiseFilter<double,uchar>::interpolate
                    (input->value, threshold->value, thresh, output->value);
        else
            utils_cv::ThresholdNoiseFilter<double,uchar>::filter
                    (input->value, threshold->value, thresh, output->value);
        break;
    }

    msg::publish(output_, output);
}

void ThresholdNoiseFilter::setup()
{
    input_      = modifier_->addInput<CvMatMessage>("unfiltered");
    threshold_  = modifier_->addInput<CvMatMessage>("weights");
    output_     = modifier_->addOutput<CvMatMessage>("filtered");
}

void ThresholdNoiseFilter::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("threshold", 0, 255, 255, 1));
    addParameter(param::ParameterFactory::declareBool("interpolate", false));
}
