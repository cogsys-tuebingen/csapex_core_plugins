///// HEADER
#include "slicing.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/node_modifier.h>
#include <utils_cv/histogram.hpp>
#include <vision_plugins_histograms/histogram_msg.h>
#include <vision_plugins_histograms/histogram_maxima_msg.h>
#include <csapex/utility/assert.h>

using namespace vision_plugins;
using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::Slicing, csapex::Node)

namespace {
    typedef std::pair<unsigned int, float> Maximum;
}

Slicing::Slicing()
{
    addParameter(param::ParameterFactory::declareRange("padding", 0, 128, 0, 1));
}

void Slicing::process()
{
    CvMatMessage::Ptr           matrix = matrix_->getMessage<CvMatMessage>();
    HistogramMaximaMessage::Ptr maxima =
            histogram_maxima_->getMessage<HistogramMaximaMessage>();
    CvMatMessage::Ptr           slices(new CvMatMessage(enc::unknown));

    apex_assert_hard(matrix->value.channels() == maxima->value.maxima.size());
    unsigned int size      = maxima->value.maxima.size();
    unsigned int padding   = readParameter<int>("padding");
    float        bin_range = maxima->value.bin_range;
    float        offset    = padding * bin_range;
    std::vector<cv::Mat>     output_channels;
    std::vector<cv::Mat>     input_channels;
    cv::split(matrix->value, input_channels);

    for(unsigned int i = 0 ; i < size ; ++i) {
        std::vector<Maximum> &channel_maxima = maxima->value.maxima.at(i);
        for(std::vector<Maximum>::iterator it = channel_maxima.begin() ;
            it != channel_maxima.end() ;
            ++it) {
            cv::Mat tmp = input_channels.at(i).clone();
            cv::threshold(tmp, tmp, it->first * bin_range - offset - 0.005, 0, CV_THRESH_TOZERO);
            cv::threshold(tmp, tmp, it->first * bin_range + offset + 0.005, 0, CV_THRESH_TOZERO_INV);
            output_channels.push_back(tmp);
        }
    }

    if(output_channels.size() > 0)
        cv::merge(output_channels, slices->value);
    slices_->publish(slices);
}

void Slicing::setup()
{
    histogram_maxima_ = modifier_->addInput<HistogramMaximaMessage>("maxima");
    matrix_           = modifier_->addInput<CvMatMessage>("image");
    slices_           = modifier_->addOutput<CvMatMessage>("slices");
}
