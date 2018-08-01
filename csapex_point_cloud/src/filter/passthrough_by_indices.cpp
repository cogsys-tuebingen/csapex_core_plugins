/// HEADER
#include "passthrough_by_indices.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_pointer_message.hpp>

/// SYSTEM
#include <pcl/filters/extract_indices.h>

CSAPEX_REGISTER_CLASS(csapex::PassThroughByIndices, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

PassThroughByIndices::PassThroughByIndices()
{
}

void PassThroughByIndices::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::factory::declareBool("keep organized", true));

}

void PassThroughByIndices::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    input_indices_ = node_modifier.addMultiInput<PointIndicesMessage, GenericVectorMessage>("Indices");

    output_pos_ = node_modifier.addOutput<PointCloudMessage>("filtered PointCloud (+)");
    output_neg_ = node_modifier.addOutput<PointCloudMessage>("filtered PointCloud (-)");
}

void PassThroughByIndices::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));

    boost::apply_visitor (PointCloudMessage::Dispatch<PassThroughByIndices>(this, msg), msg->value);
}

template <class PointT>
void PassThroughByIndices::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    pcl::PointIndicesPtr index(new pcl::PointIndices);
    if(msg::isMessage<PointIndicesMessage>(input_indices_))
    {
        PointIndicesMessage::ConstPtr indices = msg::getMessage<PointIndicesMessage>(input_indices_);
        index = indices->value;

    }
    else{
        GenericVectorMessage::ConstPtr message = msg::getMessage<GenericVectorMessage>(input_indices_);
        apex_assert(std::dynamic_pointer_cast<GenericPointerMessage<pcl::PointIndices> const>(message->nestedType()));
        for(std::size_t i = 0; i < message->nestedValueCount(); ++i){
            auto val = std::dynamic_pointer_cast<GenericPointerMessage<pcl::PointIndices> const>(message->nestedValue(i));
            index->indices.insert(index->indices.end(), val->value->indices.begin(), val->value->indices.end());
        }
    }

    // check available fields!
    pcl::ExtractIndices<PointT> pass;
//    pass.setIndices(indices->value);
    pass.setIndices(index);


    pass.setInputCloud(cloud);
    pass.setKeepOrganized(readParameter<bool>("keep organized"));

    if(msg::isConnected(output_pos_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_pos_, msg);
    }

    if(msg::isConnected(output_neg_)) {
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pass.setNegative(true);
        pass.filter(*out);
        out->header = cloud->header;

        PointCloudMessage::Ptr msg(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        msg->value = out;
        msg::publish(output_neg_, msg);
    }
}
