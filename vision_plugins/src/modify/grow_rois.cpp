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
    std::shared_ptr<std::vector<RoiMessage> const> rois_in =
            msg::getMessage<GenericVectorMessage, RoiMessage>(input_);
    std::shared_ptr<std::vector<RoiMessage>> rois_out(new std::vector<RoiMessage>);
    rois_out->assign(rois_in->begin(), rois_in->end());

    for(RoiMessage &roi : *rois_out) {
        roi.value.grow(x_, y_);
    }

    msg::publish<GenericVectorMessage, RoiMessage>(output_, rois_out);
}

void GrowROIs::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
    output_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("grown ROIs");
}
