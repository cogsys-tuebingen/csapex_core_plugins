/// HEADER
#include "merge_rois.h"

/// PROJECT
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex_opencv/cv_mat_message.h>
#include <csapex_opencv/roi_message.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <cslibs_vision/utils/rectangle_cluster.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM


CSAPEX_REGISTER_CLASS(csapex::MergeROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

MergeROIs::MergeROIs()
{
}


void MergeROIs::setup(NodeModifier& node_modifier)
{
    input_ = node_modifier.addInput<GenericVectorMessage, RoiMessage>("ROIs");
    output_ = node_modifier.addOutput<GenericVectorMessage, RoiMessage>("merged ROIs");
}

void MergeROIs::process()
{
    std::shared_ptr<std::vector<RoiMessage> const> rois =
            msg::getMessage<GenericVectorMessage, RoiMessage>(input_);

    RectangleCluster cluster;
    for(const RoiMessage &roi : *rois) {
        cluster.integrate(roi.value.rect());
    }

    std::shared_ptr<std::vector<RoiMessage>> out(new std::vector<RoiMessage>);
    for(std::vector<cv::Rect>::iterator it = cluster.begin(); it != cluster.end(); ++it) {
        RoiMessage msg;
        msg.value = Roi(*it);
        out->push_back(msg);
    }

    msg::publish<GenericVectorMessage, RoiMessage>(output_, out);
}

