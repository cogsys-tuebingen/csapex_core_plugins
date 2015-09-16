/// HEADER
#include "grow_roi.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <utils_vision/utils/rectangle_cluster.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM


CSAPEX_REGISTER_CLASS(csapex::GrowROI, csapex::Node)

using namespace csapex;
using namespace connection_types;

GrowROI::GrowROI()
{
}

void GrowROI::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("x", 0, 100, 0, 1));
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("y", 0, 100, 0, 1));
}

void GrowROI::process()
{
    RoiMessage::ConstPtr roi = msg::getMessage<RoiMessage>(input_);
    RoiMessage::Ptr out(new RoiMessage);

    out->value = roi->value;
    out->value.grow(readParameter<int>("x"), readParameter<int>("y"));
    msg::publish(output_, out);
}

void GrowROI::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<RoiMessage>("ROI");
    output_ = node_modifier.addOutput<RoiMessage>("grown ROI");
}
