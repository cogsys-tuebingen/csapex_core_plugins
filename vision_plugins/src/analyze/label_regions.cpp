/// HEADER
#include "label_regions.h"

/// PROJECT
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/io.h>
#include <utils_param/parameter_factory.h>
#include <csapex_vision/cv_mat_message.h>
#include <utils_cv/flood.h>
#include <csapex/model/node_modifier.h>

using namespace csapex;
using namespace csapex::connection_types;
using namespace vision_plugins;

CSAPEX_REGISTER_CLASS(vision_plugins::LabelRegions, csapex::Node)

LabelRegions::LabelRegions()
{
}

void LabelRegions::process()
{
#warning "FIX ENCODING"
    CvMatMessage::ConstPtr in = msg::getMessage<connection_types::CvMatMessage>(input_);
    CvMatMessage::Ptr out(new CvMatMessage(enc::unknown, in->stamp_micro_seconds));

    if(in->value.type() != CV_8UC1) {
        throw std::runtime_error("Edges should be mask with type of CV_8UC1!");
    }

    unsigned int threshold = readParameter<int>("area thresh");

    uchar   edge   = readParameter<int>("edge value");
    if(threshold > 0)
        utils_cv::label(in->value, out->value, edge, threshold);
    else
        utils_cv::label(in->value, out->value, edge);

    msg::publish(output_, out);
}

void LabelRegions::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<CvMatMessage>("edges");
    output_ = node_modifier.addOutput<CvMatMessage>("labels");
}

void LabelRegions::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("edge value", 0, 255, 255, 1));
    parameters.addParameter(param::ParameterFactory::declareRange("area thresh", 0, 1000, 0, 10));
}

