/// HEADER
#include "merge_clouds.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex/utility/register_apex_plugin.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
//#include <boost/assign.hpp>

CSAPEX_REGISTER_CLASS(csapex::MergeClouds, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;


MergeClouds::MergeClouds()
{
}

void MergeClouds::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("input count", 2, 8, 2, 1), [this](param::Parameter*) {
        updateInputs();
    });
}

void MergeClouds::setup(NodeModifier& node_modifier)
{
    out_ = node_modifier.addOutput<PointCloudMessage>("merged PointCloud");

    updateInputs();
}

void MergeClouds::process()
{
    result_.reset();

    std::string frame;
    uint64_t stamp = 0;
    std::vector<Input*> inputs = modifier_->getMessageInputs();
    for(std::size_t i = 0 ; i < inputs.size() ; i++) {
        Input *in = inputs[i];
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

void MergeClouds::updateInputs()
{
    int input_count = readParameter<int>("input count");

    std::vector<Input*> inputs = modifier_->getMessageInputs();
    int current_amount = inputs.size();

    if(current_amount > input_count) {
        for(int i = current_amount; i > input_count ; i--) {
            Input* in = inputs[i - 1];
            if(msg::isConnected(in)) {
                msg::disable(in);
            } else {
                modifier_->removeInput(msg::getUUID(in));
            }
        }
    } else {
        int to_add = input_count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(inputs[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            modifier_->addOptionalInput<PointCloudMessage>("Cloud");
        }
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
