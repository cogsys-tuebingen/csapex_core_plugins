/// HEADER
#include "histogram.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_core_plugins/vector_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_core_plugins/ros_message_conversion.h>
#include <utils_cv/histogram.hpp>
#include <utils_param/parameter_factory.h>

using namespace vision_plugins;
using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::Histogram, csapex::Node)

Histogram::Histogram() :
    bins_(256),
    uniform_(true),
    accumulate_(false)
{
    Tag::createIfNotExists("Histogram");
    addTag(Tag::get("Histogram"));
    addTag(Tag::get("Vision"));
    addTag(Tag::get("vision_plugins"));

    addParameter(param::ParameterFactory::declareRange("bins", 2, 512, 255, 1),
                 boost::bind(&Histogram::update, this));
    addParameter(param::ParameterFactory::declareBool("uniform", true),
                 boost::bind(&Histogram::update, this));
    addParameter(param::ParameterFactory::declareBool("accumulate", false),
                 boost::bind(&Histogram::update, this));

}

void Histogram::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();
    boost::shared_ptr<std::vector<CvMatMessage::Ptr> > out(new std::vector<CvMatMessage::Ptr>);

    cv::Mat mask;
    if(mask_->isConnected()) {
        CvMatMessage::Ptr mask_ptr = mask_->getMessage<connection_types::CvMatMessage>();
        mask = mask_ptr->value;
    }

    int type = in->value.type() & 7;
    std::vector<utils_cv::Range> ranges;
    std::vector<int>             bins;

    switch(type) {
    case CV_8U:
        ranges.push_back(utils_cv::make_range<unsigned char>());
        break;
    case CV_8S:
        ranges.push_back(utils_cv::make_range<signed char>());
        break;
    case CV_16U:
        ranges.push_back(utils_cv::make_range<unsigned short>());
        break;
    case CV_16S:
        ranges.push_back(utils_cv::make_range<signed short>());
        break;
    case CV_32S:
        ranges.push_back(utils_cv::make_range<int>());
        break;
    case CV_32F:
        ranges.push_back(utils_cv::make_range<float>());
        break;
    default:
        throw std::runtime_error("Unsupported cv type!");
    }

    bins.push_back(bins_);
    for(int i = 1 ; i < in->value.channels() ; ++i) {
        ranges.push_back(ranges.back());
        bins.push_back(bins.back());
    }

    std::vector<cv::Mat> histograms;
    utils_cv::histogram(in->value, histograms, mask, bins, ranges, uniform_, accumulate_);

    for(std::vector<cv::Mat>::iterator it = histograms.begin() ; it != histograms.end() ; ++it) {
        out->push_back(CvMatMessage::Ptr(new CvMatMessage(enc::unknown)));
        out->back()->value = *it;
    }

    output_->publish<GenericVectorMessage, CvMatMessage::Ptr>(out);
}

void Histogram::setup()
{
    setSynchronizedInputs(true);

    input_  = addInput<CvMatMessage>("Input");
    mask_   = addInput<CvMatMessage>("Mask", true);
    output_ = addOutput<GenericVectorMessage, CvMatMessage::Ptr>("Histograms");
    update();
}

void Histogram::update()
{
    bins_       = param<int> ("bins");
    uniform_    = param<bool>("uniform");
    accumulate_ = param<bool>("accumulate");
}

