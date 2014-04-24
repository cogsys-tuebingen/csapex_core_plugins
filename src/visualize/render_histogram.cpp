/// HEADER
#include "render_histogram.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <utils_cv/histogram.hpp>
#include <utils_param/parameter_factory.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::RenderHistogram, csapex::Node)

RenderHistogram::RenderHistogram() :
    height_(480),
    width_(640)
{
    Tag::createIfNotExists("Histogram");
    addTag(Tag::get("Histogram"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareRange("width", 200, 1000, width_, 10),
                 boost::bind(&RenderHistogram::update, this));
    addParameter(param::ParameterFactory::declareRange("height", 200, 1000, height_, 10),
                 boost::bind(&RenderHistogram::update, this));
}

void RenderHistogram::process()
{
#warning "FIX ENCODING"
    boost::shared_ptr<std::vector<CvMatMessage::Ptr> const> in = input_->getMessage<GenericVectorMessage, CvMatMessage::Ptr>();
    CvMatMessage::Ptr out(new CvMatMessage(enc::bgr));
    out->value = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(0,0,0));

    int color_count = 0;
    for(std::vector<CvMatMessage::Ptr>::const_iterator it = in->begin() ; it != in->end() ; ++it, ++color_count) {
        cv::Mat histogram = (*it)->value;
        int type = histogram.type() & 7;
        switch(type) {
        case CV_32F:
            utils_cv::render_curve<float>(histogram,
                                          utils_cv::COLOR_PALETTE.at(color_count % utils_cv::COLOR_PALETTE.size()),
                                          out->value);
            break;
        case CV_32S:
            utils_cv::render_curve<int>(histogram,
                                          utils_cv::COLOR_PALETTE.at(color_count % utils_cv::COLOR_PALETTE.size()),
                                          out->value);
            break;
        default:
            aerr << "Only 32bit float or 32bit integer histograms supported!" << std::endl;
        }
    }

    output_->publish(out);
}

void RenderHistogram::setup()
{
    setSynchronizedInputs(true);

    input_  = addInput<GenericVectorMessage, CvMatMessage::Ptr>("histograms");
    output_ = addOutput<CvMatMessage>("image");
}

void RenderHistogram::update()
{
    width_  = param<int>("width");
    height_ = param<int>("height");
}
