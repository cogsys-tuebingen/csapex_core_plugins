/// HEADER
#include "merge_rois.h"

/// PROJECT
#include <csapex_core_plugins/vector_message.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex_vision/roi_message.h>

/// PROJECT
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <utils_param/parameter_factory.h>
#include <utils/rectangle_cluster.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <boost/foreach.hpp>

CSAPEX_REGISTER_CLASS(csapex::MergeROIs, csapex::Node)

using namespace csapex;
using namespace connection_types;

MergeROIs::MergeROIs()
{
    addTag(Tag::get("Vision"));
    addTag(Tag::get("ROI"));
}

void MergeROIs::allConnectorsArrived()
{
    VectorMessage::Ptr rois = input_->getMessage<VectorMessage>();

    RectangleCluster cluster;
    for(std::vector<ConnectionType::Ptr>::const_iterator it = rois->value.begin(); it != rois->value.end(); ++it) {
        RoiMessage::Ptr roi = boost::dynamic_pointer_cast<RoiMessage>(*it);
        const Roi& r = roi->value;
        cluster.integrate(r.rect());
    }

    VectorMessage::Ptr out(new VectorMessage(RoiMessage::make()));
    for(std::vector<cv::Rect>::iterator it = cluster.begin(); it != cluster.end(); ++it) {
        RoiMessage::Ptr msg(new RoiMessage);
        msg->value = Roi(*it);
        out->value.push_back(msg);
    }

    output_->publish(out);
}

void MergeROIs::setup()
{
    setSynchronizedInputs(true);

    input_ = addInput<VectorMessage>("ROIs");
    input_->setType(VectorMessage::Ptr(new VectorMessage(RoiMessage::make())));

    output_ = addOutput<VectorMessage>("merged ROIs");
    output_->setType(VectorMessage::Ptr(new VectorMessage(RoiMessage::make())));
}
