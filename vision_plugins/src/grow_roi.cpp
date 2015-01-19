/// HEADER
#include "grow_roi.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <utils_vision/utils/rectangle_cluster.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM


CSAPEX_REGISTER_CLASS(csapex::GrowROI, csapex::Node)

using namespace csapex;
using namespace connection_types;

GrowROI::GrowROI()
{
    addParameter(param::ParameterFactory::declareRange("x", 0, 100, 0, 1));
    addParameter(param::ParameterFactory::declareRange("y", 0, 100, 0, 1));
}

void GrowROI::process()
{
    RoiMessage::ConstPtr roi = input_->getMessage<RoiMessage>();
    RoiMessage::Ptr out(new RoiMessage);

    out->value = roi->value;
    out->value.grow(readParameter<int>("x"), readParameter<int>("y"));
    output_->publish(out);
}

void GrowROI::setup()
{
    input_ = modifier_->addInput<RoiMessage>("ROI");
    output_ = modifier_->addOutput<RoiMessage>("grown ROI");
}
