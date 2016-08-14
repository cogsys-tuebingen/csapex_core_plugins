/// HEADER
#include "render_histogram.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <cslibs_vision/utils/histogram.hpp>
#include <csapex_vision_histograms/histogram_msg.h>
#include <csapex_vision_histograms/histogram_maxima_msg.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::RenderHistogram, csapex::Node)

RenderHistogram::RenderHistogram() :
    height_(480),
    width_(640)
{
}

void RenderHistogram::process()
{
    HistogramMessage::ConstPtr in = msg::getMessage<HistogramMessage>(input_);
    HistogramMaximaMessage::ConstPtr maxima;
    if(msg::hasMessage(maxima_)) {
        maxima = msg::getMessage<HistogramMaximaMessage>(maxima_);
        if(maxima->value.maxima.size() != in->value.histograms.size()) {
            throw std::runtime_error("Histograms and maxima entries must have the same size!");
        }
    }

    CvMatMessage::Ptr out(new CvMatMessage(enc::bgr, in->stamp_micro_seconds));
    out->value = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(0,0,0));

    int line_width = readParameter<int>("line width");

    if(maxima.get() == nullptr) {
        int color_count = 0;
        for(std::vector<cv::Mat>::const_iterator it = in->value.histograms.begin() ;
            it != in->value.histograms.end() ;
            ++it, ++color_count) {
            int type = it->type();
            switch(type) {
            case CV_32FC1:
                cslibs_vision::histogram::render_curve<float>(*it,
                                                         cslibs_vision::histogram::COLOR_PALETTE.at
                                                         (color_count % cslibs_vision::histogram::COLOR_PALETTE.size()),
                                                         line_width,
                                                         out->value);
                break;
            case CV_32SC1:
                cslibs_vision::histogram::render_curve<int>(  *it,
                                                         cslibs_vision::histogram::COLOR_PALETTE.at
                                                         (color_count % cslibs_vision::histogram::COLOR_PALETTE.size()),
                                                         line_width,
                                                         out->value);
                break;
            default:
                throw std::runtime_error("Only 32bit float or 32bit integer histograms supported!");
            }
        }
    } else {
        const std::vector<cv::Mat> &histograms = in->value.histograms;
        for(unsigned int i = 0 ; i < histograms.size() ; ++i) {
            int type = histograms.at(i).type() & 7;
            switch(type) {
            case CV_32F:
                cslibs_vision::histogram::render_curve<float>(histograms.at(i),
                                                         maxima->value.maxima.at(i),
                                                         cslibs_vision::histogram::COLOR_PALETTE.at
                                                         (i % cslibs_vision::histogram::COLOR_PALETTE.size()),
                                                         line_width,
                                                         5,
                                                         out->value);
                break;
            case CV_32S:
                cslibs_vision::histogram::render_curve<int>(histograms.at(i),
                                                       maxima->value.maxima.at(i),
                                                       cslibs_vision::histogram::COLOR_PALETTE.at
                                                       (i % cslibs_vision::histogram::COLOR_PALETTE.size()),
                                                       line_width,
                                                       5,
                                                       out->value);
                break;
            default:
                throw std::runtime_error("Only 32bit float or 32bit integer histograms supported!");

            }
        }
    }

    msg::publish(output_, out);
}

void RenderHistogram::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("width", 200, 1000, width_, 10),
                 std::bind(&RenderHistogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("height", 200, 1000, height_, 10),
                 std::bind(&RenderHistogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("line width", 1, 10, 1, 1));
}

void RenderHistogram::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<HistogramMessage>("histograms");
    output_ = node_modifier.addOutput<CvMatMessage>("image");
    maxima_ = node_modifier.addOptionalInput<HistogramMaximaMessage>("maxima");
}

void RenderHistogram::update()
{
    width_  = readParameter<int>("width");
    height_ = readParameter<int>("height");
}
