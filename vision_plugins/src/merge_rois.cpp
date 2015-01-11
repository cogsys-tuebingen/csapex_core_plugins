/// HEADER
#include "merge_rois.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <utils_param/parameter_factory.h>
#include <utils_vision/utils/rectangle_cluster.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::MergeROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

MergeROIs::MergeROIs()
{
}

void MergeROIs::process()
{
    VectorMessage::ConstPtr rois = input_->getMessage<VectorMessage>();

    RectangleCluster cluster;
    for(std::vector<ConnectionType::Ptr>::const_iterator it = rois->value.begin(); it != rois->value.end(); ++it) {
        RoiMessage::Ptr roi = boost::dynamic_pointer_cast<RoiMessage>(*it);
        const Roi& r = roi->value;
        cluster.integrate(r.rect());
    }

    VectorMessage::Ptr out(VectorMessage::make<RoiMessage>());
    for(std::vector<cv::Rect>::iterator it = cluster.begin(); it != cluster.end(); ++it) {
        RoiMessage::Ptr msg(new RoiMessage);
        msg->value = Roi(*it);
        out->value.push_back(msg);
    }

    output_->publish(out);
}

void MergeROIs::setup()
{
    input_ = modifier_->addInput<VectorMessage, RoiMessage>("ROIs");
    output_ = modifier_->addOutput<VectorMessage, RoiMessage>("merged ROIs");
}
