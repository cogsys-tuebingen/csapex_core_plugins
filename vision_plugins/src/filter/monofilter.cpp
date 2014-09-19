/// HEADER
#include "monofilter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_vision/utils/histogram.hpp>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::MonoFilter, csapex::Node)

MonoFilter::MonoFilter()
{
}

void MonoFilter::process()
{
    CvMatMessage::Ptr in = input_->getMessage<connection_types::CvMatMessage>();

    if(!in->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono));

    out->value    = in->value.clone();

    for(int i = 0 ; i < out->value.rows ; ++i) {
        for(int j = 0 ; j < out->value.cols ; ++j) {
            uchar val = out->value.at<uchar>(i,j);
            bool  in_range = val < min_ || val > max_;
            if(invert_ ? !in_range : in_range ) {
                out->value.at<uchar>(i,j) = def_;
            }
        }
    }

    output_->publish(out);
}


void MonoFilter::setup()
{
    input_ = modifier_->addInput<CvMatMessage>("original");
    output_ = modifier_->addOutput<CvMatMessage>("filtered");
    update();
}

void MonoFilter::setupParameters()
{
    addParameter(param::ParameterFactory::declareRange("min", 0, 255, 0, 1),
                 boost::bind(&MonoFilter::update, this));
    addParameter(param::ParameterFactory::declareRange("max", 0, 255, 255, 1),
                 boost::bind(&MonoFilter::update, this));
    addParameter(param::ParameterFactory::declareRange("def", 0, 255, 255, 1),
                 boost::bind(&MonoFilter::update, this));
    addParameter(param::ParameterFactory::declareBool("invert", false),
                 boost::bind(&MonoFilter::update, this));
}

void MonoFilter::update()
{
    max_    = readParameter<int>("max");
    min_    = readParameter<int>("min");
    def_    = readParameter<int>("def");
    invert_ = readParameter<bool>("invert");
}
