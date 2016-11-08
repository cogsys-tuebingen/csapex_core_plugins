/// HEADER
#include "merge_clouds.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/point_cloud_message.h>

CSAPEX_REGISTER_CLASS(csapex::MergeClouds, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MergeClouds::MergeClouds()
{
}

Input* MergeClouds::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional)
{
    return VariadicInputs::createVariadicInput(connection_types::makeEmpty<connection_types::PointCloudMessage>(), label.empty() ? "Cloud" : label, getVariadicInputCount() == 0 ? false : true);
}

void MergeClouds::setupParameters(Parameterizable& parameters)
{
    setupVariadicParameters(parameters);
}

void MergeClouds::setup(NodeModifier& node_modifier)
{
    setupVariadic(node_modifier);

    out_ = node_modifier.addOutput<PointCloudMessage>("merged PointCloud");
}

void MergeClouds::process()
{
    result_.reset();

    std::string frame;
    uint64_t stamp = 0;
    std::vector<InputPtr> inputs = node_modifier_->getMessageInputs();
    for(std::size_t i = 0 ; i < inputs.size() ; i++) {
        Input *in = inputs[i].get();
        if(msg::hasMessage(in)) {

            PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in));
            if(frame.empty()) {
                frame = msg->frame_id;
            }
            if(stamp == 0) {
                stamp = msg->stamp_micro_seconds;
            }
            boost::apply_visitor (PointCloudMessage::Dispatch<MergeClouds>(this, msg), msg->value);
        }
    }

    if(result_) {
        msg::publish(out_, result_);
    }
}

template <class PointT>
void MergeClouds::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if(!result_) {
        result_ = std::make_shared<connection_types::PointCloudMessage>(cloud->header.frame_id, cloud->header.stamp);
        result_->value = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    }

    typename pcl::PointCloud<PointT>::Ptr res_cloud = boost::get<typename pcl::PointCloud<PointT>::Ptr>(result_->value);

    res_cloud->points.reserve(res_cloud->points.size() + cloud->points.size());
    for(const auto& pt : cloud->points) {
        res_cloud->points.push_back(pt);
    }
}
