/// HEADER
#include "histogram.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <utils_cv/histogram.hpp>
#include <vision_plugins_histograms/histogram_msg.h>
#include <csapex/model/node_modifier.h>

using namespace vision_plugins;
using namespace csapex;
using namespace csapex::connection_types;

CSAPEX_REGISTER_CLASS(vision_plugins::Histogram, csapex::Node)

Histogram::Histogram() :
    bins_(256),
    last_type_(CV_8U),
    uniform_(true),
    accumulate_(false),
    min_max_(false),
    min_max_global_(false),
    min_max_value_(std::make_pair<float, float>(std::numeric_limits<float>::max(),
                                                std::numeric_limits<float>::min()))
{
    addParameter(param::ParameterFactory::declareRange("bins", 2, 512, 255, 1),
                 boost::bind(&Histogram::update, this));
    addParameter(param::ParameterFactory::declareBool("uniform", uniform_),
                 boost::bind(&Histogram::update, this));
    addParameter(param::ParameterFactory::declareBool("accumulate", accumulate_),
                 boost::bind(&Histogram::update, this));
    addParameter(param::ParameterFactory::declareBool("min max range", false),
                 boost::bind(&Histogram::update, this));
    addParameter(param::ParameterFactory::declareBool("global min max", false),
                 boost::bind(&Histogram::update, this));
}

void Histogram::process()
{
    CvMatMessage::Ptr in = input_->getMessage<CvMatMessage>();
    HistogramMessage::Ptr out(new HistogramMessage);

    cv::Mat mask;
    if(mask_->isConnected()) {
        CvMatMessage::Ptr mask_ptr = mask_->getMessage<CvMatMessage>();
        mask = mask_ptr->value;
    }

    int type = in->value.type() & 7;
    std::vector<utils_cv::histogram::Range> ranges;
    std::vector<int>             bins;

    if(min_max_ && type != last_type_)
        resetMinMax();

    utils_cv::histogram::Range range;
    switch(type) {
    case CV_8U:
        if(min_max_)
            range = utils_cv::histogram::
                    make_min_max_range<unsigned char>(in->value, mask);
        else
            range = utils_cv::histogram::
                    make_range<unsigned char>();
        break;
    case CV_8S:
        if(min_max_)
            range = utils_cv::histogram::
                    make_min_max_range<signed char>(in->value, mask);
        else
            range = utils_cv::histogram::
                    make_range<signed char>();
        break;
    case CV_16U:
        if(min_max_)
            range = utils_cv::histogram::
                    make_min_max_range<unsigned short>(in->value, mask);
        else
            range = utils_cv::histogram::
                    make_range<unsigned short>();
        break;
    case CV_16S:
        if(min_max_)
            range = utils_cv::histogram::
                    make_min_max_range<signed short>(in->value, mask);
        else
            range = utils_cv::histogram::
                    make_range<signed short>();
        break;
    case CV_32S:
        if(min_max_)
            range = utils_cv::histogram::
                    make_min_max_range<int>(in->value, mask);
        else
            range = utils_cv::histogram::
                    make_range<int>();
        break;
    case CV_32F:
        if(min_max_)
            range = utils_cv::histogram::
                    make_min_max_range<float>(in->value, mask);
        else
            range = utils_cv::histogram::
                    make_range<float>();
        break;
    default:
        throw std::runtime_error("Unsupported cv type!");
    }

    last_type_ = type;

    if(min_max_global_) {
        if(range.first < min_max_value_.first)
            min_max_value_.first  = range.first;
        if(range.second > min_max_value_.second)
            min_max_value_.second = range.second;

        range = min_max_value_;
    }

    for(int i = 0 ; i < in->value.channels() ; ++i) {
        ranges.push_back(range);
        bins.push_back(bins_);
    }

    std::vector<cv::Mat> histograms;
    utils_cv::histogram::histogram
            (in->value, histograms, mask, bins, ranges, uniform_, accumulate_);
    for(std::vector<cv::Mat>::iterator it = histograms.begin() ; it != histograms.end() ; ++it) {
        out->value.histograms.push_back(*it);
    }
    out->value.range     = range;
    out->value.bin_range = (range.second - range.first) / (float) bins_;
    output_->publish(out);
}

void Histogram::setup()
{
    input_  = modifier_->addInput<CvMatMessage>("input");
    mask_   = modifier_->addInput<CvMatMessage>("mask", true);
    output_ = modifier_->addOutput<HistogramMessage>("histograms");
    update();
}

void Histogram::update()
{

    bins_           = readParameter<int> ("bins");
    uniform_        = readParameter<bool>("uniform");
    accumulate_     = readParameter<bool>("accumulate");
    min_max_        = readParameter<bool>("min max range");
    min_max_global_ = readParameter<bool>("global min max") && min_max_;
    setParameterEnabled("global min max", min_max_);
    if(min_max_global_) {
        resetMinMax();
    }
}

void Histogram::resetMinMax()
{
    min_max_value_ = std::make_pair<float, float>(std::numeric_limits<float>::max(),
                                                  std::numeric_limits<float>::min());
}

