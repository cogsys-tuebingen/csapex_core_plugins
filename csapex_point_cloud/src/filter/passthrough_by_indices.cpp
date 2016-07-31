/// HEADER
#include "passthrough_by_indices.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/point_cloud_message.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/interval_parameter.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

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
    parameters.addParameter(csapex::param::ParameterFactory::declareBool("keep organized", true));

}

void PassThroughByIndices::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    input_indices_ = node_modifier.addInput<PointIndecesMessage>("Indices");

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
    PointIndecesMessage::ConstPtr indices = msg::getMessage<PointIndecesMessage>(input_indices_);

    // check available fields!
    pcl::ExtractIndices<PointT> pass;
    pass.setIndices(indices->value);

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
