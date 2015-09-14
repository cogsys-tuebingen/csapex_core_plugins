/// HEADER
#include "statistical_outlier_removal.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/indeces_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Wsign-compare"
#endif //__clang__
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#if __clang__
#pragma clang diagnostic pop#include <pcl/filters/statistical_outlier_removal.h>
#endif //__clang__

CSAPEX_REGISTER_CLASS(csapex::StatisticalOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

StatisticalOutlierRemoval::StatisticalOutlierRemoval()
{
}

void StatisticalOutlierRemoval::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(param::ParameterFactory::declareRange("mean k", 1, 100, 2, 1));
    parameters.addParameter(param::ParameterFactory::declareBool ("keep organized", false));
    parameters.addParameter(param::ParameterFactory::declareBool ("negate", false));
    parameters.addParameter(param::ParameterFactory::declareRange("std dev threshold", 0.0, 10.0, 0.1, 0.1));
}

void StatisticalOutlierRemoval::setup(NodeModifier& node_modifier)
{
    input_cloud_    = node_modifier.addInput<PointCloudMessage>   ("PointCloud");
    input_indeces_  = node_modifier.addOptionalInput<PointIndecesMessage> ("Indeces");
    output_cloud_   = node_modifier.addOutput<PointCloudMessage>  ("Pointcloud");
    output_indeces_ = node_modifier.addOutput<PointIndecesMessage>("Indeces");
}

void StatisticalOutlierRemoval::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<StatisticalOutlierRemoval>(this, msg), msg->value);
}

template <class PointT>
void StatisticalOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    bool indeces_out = msg::isConnected(output_indeces_);
    bool cloud_out   = msg::isConnected(output_cloud_);

    if(!indeces_out && !cloud_out)
        return;

    int    mean_k             = readParameter<int>("mean k");
    bool   keep_organized     = readParameter<bool>("keep organized");
    bool   negative           = readParameter<bool>("negate");
    double std_dev_mul_thresh = readParameter<double>("std dev threshold");

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setKeepOrganized(keep_organized);
    sor.setMeanK(mean_k);
    sor.setNegative(negative);
    sor.setStddevMulThresh(std_dev_mul_thresh);
    if(msg::hasMessage(input_indeces_)) {
        PointIndecesMessage::ConstPtr indeces(msg::getMessage<PointIndecesMessage>(input_indeces_));
        sor.setIndices(indeces->value);
    }
    if(cloud_out) {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        sor.filter(*cloud_filtered);
        PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        out->value = cloud_filtered;
        msg::publish(output_cloud_, out);
    }
    if(indeces_out) {
        PointIndecesMessage::Ptr indeces_filtered(new PointIndecesMessage);
        indeces_filtered->value->header = cloud->header;
        sor.filter(indeces_filtered->value->indices);
        msg::publish(output_indeces_, indeces_filtered);
    }
}
