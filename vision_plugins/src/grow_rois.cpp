/// HEADER
#include "grow_rois.h"

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


CSAPEX_REGISTER_CLASS(csapex::GrowROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

GrowROIs::GrowROIs()
{
}

void GrowROIs::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("x", 0, 100, 0, 1), x_);
    parameters.addParameter(csapex::param::ParameterFactory::declareRange("y", 0, 100, 0, 1), y_);
}

void GrowROIs::process()
{
    VectorMessage::ConstPtr rois_in = msg::getMessage<VectorMessage>(input_);
    VectorMessage::Ptr      rois_out(VectorMessage::make<RoiMessage>());

    for(const auto roi : rois_in->value) {
        RoiMessage::ConstPtr roi_in =
                std::dynamic_pointer_cast<RoiMessage const>(roi);
        RoiMessage::Ptr roi_out = roi_in->cloneAs<RoiMessage>();
        roi_out->value.grow(x_, y_);
        rois_out->value.emplace_back(roi_out);
    }
    msg::publish(output_, rois_out);
}

void GrowROIs::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<VectorMessage, RoiMessage>("ROI");
    output_ = node_modifier.addOutput<VectorMessage, RoiMessage>("grown ROI");
}
