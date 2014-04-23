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
}


void ThresholdNoiseFilter::process()
{
    utils_cv::ThresholdNoiseFilter<float> T;
}

void ThresholdNoiseFilter::setup()
{
    input_      = addInput<CvMatMessage>("unfiltered");
    threshold_  = addInput<CvMatMessage>("weights");
    output_     = addOutput<CvMatMessage>("filtered");

}
