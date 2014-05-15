/// HEADER
#include "histogram_maxima.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <utils_param/parameter_factory.h>

#include <utils_cv/histogram.hpp>
#include <vision_plugins_histograms/histogram_msg.h>
#include <vision_plugins_histograms/histogram_maxima_msg.h>

using namespace vision_plugins;
using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::HistogramMaxima, csapex::Node)

HistogramMaxima::HistogramMaxima()
{
    Tag::createIfNotExists("Histogram");
    addTag(Tag::get("Histogram"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareRange("k", 1, 128, 2, 1));
    addParameter(param::ParameterFactory::declareRange("thresh", 0, 1000, 0, 1));
}

void HistogramMaxima::process()
{
    HistogramMessage::Ptr       input  = histograms_->getMessage<HistogramMessage>();
    HistogramMaximaMessage::Ptr output(new HistogramMaximaMessage);

    unsigned int count = input->value.histograms.size();
    output->value.maxima.resize(count);
    output->value.bin_range = input->value.bin_range;

    unsigned int k = param<int>("k");
    float thresh   = param<int>("thresh");
    for(unsigned int i = 0 ; i < count ; ++i) {
        cv::Mat &src = input->value.histograms.at(i);
        int type = src.type() & 7;
        switch(type) {
        case CV_32F:
            utils_cv::histogram::find_maxima1D<float>(src,
                                                      k,
                                                      thresh,
                                                      output->value.maxima.at(i));
            break;
        case CV_32S:
            utils_cv::histogram::find_maxima1D<int>(src,
                                                    k,
                                                    thresh,
                                                    output->value.maxima.at(i));
            break;
        default:
            throw std::runtime_error("Only 32bit float or 32bit integer histograms supported!");
        }
    }
    maxima_->publish(output);
}

void HistogramMaxima::setup()
{
    histograms_ = addInput<HistogramMessage>("histograms");
    maxima_     = addOutput<HistogramMaximaMessage>("maxima");
}
