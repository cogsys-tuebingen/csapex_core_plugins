/// HEADER
#include "threshold_noise_filter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>

#include <utils_cv/noise_filter.hpp>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::ThresholdNoiseFilter, csapex::Node)

ThresholdNoiseFilter::ThresholdNoiseFilter()
{
    addTag(Tag::get("Experimental"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareRange("threshold", 0, 255, 255, 1));
}


void ThresholdNoiseFilter::process()
{
    CvMatMessage::Ptr input     = input_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr threshold = threshold_->getMessage<CvMatMessage>();
    CvMatMessage::Ptr output(new CvMatMessage(input->getEncoding()));

    if(threshold->getEncoding() != enc::mono) {
        throw std::runtime_error("Threshold needs to be mono!");
    }


    uchar thresh = param<int>("threshold");
    int   type   = input->value.type();
    switch(type) {
    case CV_8UC1:
        utils_cv::ThresholdNoiseFilter<uchar,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    case CV_8SC1:
        utils_cv::ThresholdNoiseFilter<char,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    case CV_16UC1:
        utils_cv::ThresholdNoiseFilter<ushort,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    case CV_16SC1:
        utils_cv::ThresholdNoiseFilter<short,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    case CV_32SC1:
        utils_cv::ThresholdNoiseFilter<int,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    case CV_32FC1:
        utils_cv::ThresholdNoiseFilter<float,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    case CV_64FC1:
        utils_cv::ThresholdNoiseFilter<double,uchar>::apply(input->value, threshold->value, thresh, output->value);
        break;
    }

    output_->publish(output);
}

void ThresholdNoiseFilter::setup()
{
    input_      = addInput<CvMatMessage>("unfiltered");
    threshold_  = addInput<CvMatMessage>("weights");
    output_     = addOutput<CvMatMessage>("filtered");

}
