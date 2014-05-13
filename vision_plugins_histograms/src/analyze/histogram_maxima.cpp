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
}

void HistogramMaxima::process()
{
     HistogramMessage::Ptr       input  = histograms_->getMessage<HistogramMessage>();
     HistogramMaximaMessage::Ptr output(new HistogramMaximaMessage);

     unsigned int count = input->value.histograms.size();
     output->value.maxima.resize(count);

     for(unsigned int i = 0 ; i < count ; ++i) {
         cv::Mat &histogram = input->value.histograms.at(i);
        /// apply find maximum
     }

}

void HistogramMaxima::setup()
{
    histograms_ = addInput<HistogramMessage>("histograms");
    maxima_     = addOutput<HistogramMaximaMessage>("maxima");
}
