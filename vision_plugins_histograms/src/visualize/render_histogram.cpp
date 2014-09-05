/// HEADER
#include "render_histogram.h"

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <utils_cv/histogram.hpp>
#include <vision_plugins_histograms/histogram_msg.h>
#include <vision_plugins_histograms/histogram_maxima_msg.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::RenderHistogram, csapex::Node)

RenderHistogram::RenderHistogram() :
    height_(480),
    width_(640)
{
    addParameter(param::ParameterFactory::declareRange("width", 200, 1000, width_, 10),
                 boost::bind(&RenderHistogram::update, this));
    addParameter(param::ParameterFactory::declareRange("height", 200, 1000, height_, 10),
                 boost::bind(&RenderHistogram::update, this));
    addParameter(param::ParameterFactory::declareRange("line width", 1, 10, 1, 1));
}

void RenderHistogram::process()
{

#warning "FIX ENCODING"

    HistogramMessage::Ptr in    = input_->getMessage<HistogramMessage>();
    HistogramMaximaMessage::Ptr maxima;
    if(maxima_->hasMessage()) {
        maxima = maxima_->getMessage<HistogramMaximaMessage>();
        if(maxima->value.maxima.size() != in->value.histograms.size()) {
            throw std::runtime_error("Histograms and maxima entries must have the same size!");
        }
    }

    CvMatMessage::Ptr out(new CvMatMessage(enc::bgr));
    out->value = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(0,0,0));

    int line_width = readParameter<int>("line width");

    if(maxima.get() == NULL) {
        int color_count = 0;
        for(std::vector<cv::Mat>::const_iterator it = in->value.histograms.begin() ;
            it != in->value.histograms.end() ;
            ++it, ++color_count) {
            int type = it->type() & 7;
            switch(type) {
            case CV_32F:
                utils_cv::histogram::render_curve<float>(*it,
                                                         utils_cv::histogram::COLOR_PALETTE.at
                                                         (color_count % utils_cv::histogram::COLOR_PALETTE.size()),
                                                         line_width,
                                                         out->value);
                break;
            case CV_32S:
                utils_cv::histogram::render_curve<int>(  *it,
                                                         utils_cv::histogram::COLOR_PALETTE.at
                                                         (color_count % utils_cv::histogram::COLOR_PALETTE.size()),
                                                         line_width,
                                                         out->value);
            default:
                throw std::runtime_error("Only 32bit float or 32bit integer histograms supported!");
            }
        }
    } else {
        std::vector<cv::Mat> &histograms = in->value.histograms;
        for(unsigned int i = 0 ; i < histograms.size() ; ++i) {
            int type = histograms.at(i).type() & 7;
            switch(type) {
            case CV_32F:
                utils_cv::histogram::render_curve<float>(histograms.at(i),
                                                         maxima->value.maxima.at(i),
                                                         utils_cv::histogram::COLOR_PALETTE.at
                                                         (i % utils_cv::histogram::COLOR_PALETTE.size()),
                                                         line_width,
                                                         5,
                                                         out->value);
                break;
            case CV_32S:
                utils_cv::histogram::render_curve<int>(histograms.at(i),
                                                       maxima->value.maxima.at(i),
                                                       utils_cv::histogram::COLOR_PALETTE.at
                                                       (i % utils_cv::histogram::COLOR_PALETTE.size()),
                                                       line_width,
                                                       5,
                                                       out->value);
            default:
                throw std::runtime_error("Only 32bit float or 32bit integer histograms supported!");

            }
        }
    }

    output_->publish(out);
}

void RenderHistogram::setup()
{
    input_  = modifier_->addInput<HistogramMessage>("histograms");
    output_ = modifier_->addOutput<CvMatMessage>("image");
    maxima_ = modifier_->addOptionalInput<HistogramMaximaMessage>("maxima");
}

void RenderHistogram::update()
{
    width_  = readParameter<int>("width");
    height_ = readParameter<int>("height");
}
