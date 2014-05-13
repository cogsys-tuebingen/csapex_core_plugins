/// HEADER
#include "grow_roi.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>
#include <utils_vision/utils/rectangle_cluster.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::GrowROI, csapex::Node)

using namespace csapex;
using namespace connection_types;

GrowROI::GrowROI()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("ROI"));

    addParameter(param::ParameterFactory::declareRange("x", 0, 100, 0, 1));
    addParameter(param::ParameterFactory::declareRange("y", 0, 100, 0, 1));
}

void GrowROI::process()
{
    RoiMessage::Ptr roi = input_->getMessage<RoiMessage>();
    RoiMessage::Ptr out(new RoiMessage);

    out->value = roi->value;
    out->value.grow(param<int>("x"), param<int>("y"));
    output_->publish(out);
}

void GrowROI::setup()
{
    input_ = addInput<RoiMessage>("ROI");
    output_ = addOutput<RoiMessage>("grown ROI");
}
