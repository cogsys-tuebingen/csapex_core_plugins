/// HEADER
#include "monofilter.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex_opencv/cv_mat_message.h>
#include <cslibs_vision/utils/histogram.hpp>
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
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);

    if(!in->hasChannels(1, CV_8U)) {
        throw std::runtime_error("image must be one channel grayscale.");
    }

    CvMatMessage::Ptr out(new connection_types::CvMatMessage(enc::mono, in->stamp_micro_seconds));

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

    msg::publish(output_, out);
}


void MonoFilter::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("original");
    output_ = node_modifier.addOutput<CvMatMessage>("filtered");
    update();
}

void MonoFilter::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("min", 0, 255, 0, 1),
                 std::bind(&MonoFilter::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("max", 0, 255, 255, 1),
                 std::bind(&MonoFilter::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("def", 0, 255, 255, 1),
                 std::bind(&MonoFilter::update, this));
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("invert", false),
                 std::bind(&MonoFilter::update, this));
}

void MonoFilter::update()
{
    max_    = readParameter<int>("max");
    min_    = readParameter<int>("min");
    def_    = readParameter<int>("def");
    invert_ = readParameter<bool>("invert");
}
