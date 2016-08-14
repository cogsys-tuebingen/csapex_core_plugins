/// HEADER
#include "histogram.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <cslibs_vision/utils/histogram.hpp>
#include <csapex_vision_histograms/histogram_msg.h>
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
    append_(false),
    min_max_value_(std::make_pair<float, float>(std::numeric_limits<float>::max(),
                                                std::numeric_limits<float>::min()))
{
}

void Histogram::process()
{
    CvMatMessage::ConstPtr in = msg::getMessage<CvMatMessage>(input_);
    HistogramMessage::Ptr out(new HistogramMessage);

    append_         = readParameter<bool>("append histograms");

    cv::Mat mask;
    if(msg::hasMessage(mask_)) {
        CvMatMessage::ConstPtr mask_ptr = msg::getMessage<CvMatMessage>(mask_);
        mask = mask_ptr->value;
    }

    int type = in->value.type() & 7;
    std::vector<int> bins;

    if(min_max_ && type != last_type_)
        resetMinMax();

    cslibs_vision::histogram::Rangef range;
    switch(type) {
    case CV_8U:
        if(min_max_)
            range = cslibs_vision::histogram::
                    make_min_max_range<unsigned char>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<unsigned char>();
        break;
    case CV_8S:
        if(min_max_)
            range = cslibs_vision::histogram::
                    make_min_max_range<signed char>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<signed char>();
        break;
    case CV_16U:
        if(min_max_)
            range = cslibs_vision::histogram::
                    make_min_max_range<unsigned short>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<unsigned short>();
        break;
    case CV_16S:
        if(min_max_)
            range = cslibs_vision::histogram::
                    make_min_max_range<signed short>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<signed short>();
        break;
    case CV_32S:
        if(min_max_)
            range = cslibs_vision::histogram::
                    make_min_max_range<int>(in->value, mask);
        else
            range = cslibs_vision::histogram::
                    make_range<int>();
        break;
    case CV_32F:
        if(min_max_)
            range = cslibs_vision::histogram::
                    make_min_max_range<float>(in->value, mask);
        else
            range = cslibs_vision::histogram::
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

    std::vector<cslibs_vision::histogram::Rangef> ranges (in->value.channels(), range);
    bins.resize(in->value.channels(), bins_);

    std::vector<cv::Mat> histograms;
    cslibs_vision::histogram::histogram
            (in->value, histograms, mask, bins, ranges, uniform_, accumulate_);

    if(append_) {
        int length = histograms.front().rows;
        cv::Mat all(histograms.size() * length, 1, CV_32FC1, cv::Scalar::all(0));
        for(unsigned int i = 0 ; i < histograms.size() ; ++i) {
            cv::Mat rows(all, cv::Rect(0, i * length, 1, length));
            histograms.at(i).copyTo(rows);
        }
        out->value.histograms.push_back(all);
        out->value.ranges.resize(1, range);
    } else {
        for(std::vector<cv::Mat>::iterator it = histograms.begin() ; it != histograms.end() ; ++it) {
            out->value.histograms.push_back(*it);
        }
        out->value.ranges = ranges;
    }

    msg::publish(output_, out);
}

void Histogram::setup(NodeModifier& node_modifier)
{
    input_  = node_modifier.addInput<CvMatMessage>("input");
    mask_   = node_modifier.addOptionalInput<CvMatMessage>("mask");
    output_ = node_modifier.addOutput<HistogramMessage>("histograms");
    update();
}

void Histogram::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("bins", 2, 512, 255, 1),
                            std::bind(&Histogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("uniform", uniform_),
                            std::bind(&Histogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("accumulate", accumulate_),
                            std::bind(&Histogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("min max range", false),
                            std::bind(&Histogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("global min max", false),
                            std::bind(&Histogram::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("append histograms", false),
                            std::bind(&Histogram::update, this));
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

