/// HEADER
#include "grow_roi.h"

/// PROJECT
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <cslibs_vision/utils/rectangle_cluster.h>

/// SYSTEM

CSAPEX_REGISTER_CLASS(csapex::GrowROI, csapex::Node)

using namespace csapex;
using namespace connection_types;

GrowROI::GrowROI()
{
}

void GrowROI::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("x", 0, 100, 0, 1), x_);
    parameters.addParameter(csapex::param::factory::declareRange("y", 0, 100, 0, 1), y_);
}

void GrowROI::process()
{
    RoiMessage::ConstPtr roi = msg::getMessage<RoiMessage>(input_);
    RoiMessage::Ptr out(new RoiMessage);

    out->value = roi->value;
    out->value.grow(x_, y_);
    msg::publish(output_, out);
}

void GrowROI::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<RoiMessage>("ROI");
    output_ = node_modifier.addOutput<RoiMessage>("grown ROI");
}
