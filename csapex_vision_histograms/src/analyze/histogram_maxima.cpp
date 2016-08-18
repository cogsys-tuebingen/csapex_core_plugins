/// HEADER
#include "histogram_maxima.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <cslibs_vision/utils/histogram.hpp>
#include <csapex_vision_histograms/histogram_msg.h>
#include <csapex_vision_histograms/histogram_maxima_msg.h>

using namespace csapex;
using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(csapex::HistogramMaxima, csapex::Node)

HistogramMaxima::HistogramMaxima()
{
}

void HistogramMaxima::process()
{
    HistogramMessage::ConstPtr  input  = msg::getMessage<HistogramMessage>(histograms_);
    HistogramMaximaMessage::Ptr output(new HistogramMaximaMessage);

    unsigned int count = input->value.histograms.size();
    output->value.maxima.resize(count);

    unsigned int k = readParameter<int>("k");
    float thresh   = readParameter<int>("thresh");
    for(unsigned int i = 0 ; i < count ; ++i) {
        const cv::Mat &src = input->value.histograms.at(i);
        int type = src.type() & 7;
        switch(type) {
        case CV_32F:
            cslibs_vision::histogram::find_maxima1D<float>(src,
                                                          k,
                                                          thresh,
                                                          output->value.maxima.at(i));
            break;
        case CV_32S:
            cslibs_vision::histogram::find_maxima1D<int>(src,
                                                        k,
                                                        thresh,
                                                        output->value.maxima.at(i));
            break;
        default:
            throw std::runtime_error("Only 32bit float or 32bit integer histograms supported!");
        }
    }
    msg::publish(maxima_, output);
}

void HistogramMaxima::setup(NodeModifier& node_modifier)
{
    histograms_ = node_modifier.addInput<HistogramMessage>("histograms");
    maxima_     = node_modifier.addOutput<HistogramMaximaMessage>("maxima");
}

void HistogramMaxima::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("k", 1, 128, 2, 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("thresh", 0, 1000, 0, 1));
}
