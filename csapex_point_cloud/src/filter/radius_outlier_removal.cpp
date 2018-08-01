/// HEADER
#include "radius_outlier_removal.h"

/// PROJECT
#include <csapex/msg/io.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM

#include <csapex/utility/suppress_warnings_start.h>
    #include <pcl/point_types.h>
    #include <pcl/filters/radius_outlier_removal.h>
#include <csapex/utility/suppress_warnings_end.h>

CSAPEX_REGISTER_CLASS(csapex::RadiusOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

RadiusOutlierRemoval::RadiusOutlierRemoval()
{
}

void RadiusOutlierRemoval::setupParameters(Parameterizable &parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("min neighbours", 1, 1000, 2, 1));
    parameters.addParameter(csapex::param::factory::declareBool ("keep organized", false));
    parameters.addParameter(csapex::param::factory::declareBool ("negate", false));
    parameters.addParameter(csapex::param::factory::declareRange("search radius", 0.0, 30.0, 0.8, 0.1));
}

void RadiusOutlierRemoval::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    indices_input_ = node_modifier.addOptionalInput<PointIndicesMessage>("indices");
    output_cloud_ = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    output_indices_ = node_modifier.addOutput<PointIndicesMessage>("indices");
}

void RadiusOutlierRemoval::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
    boost::apply_visitor (PointCloudMessage::Dispatch<RadiusOutlierRemoval>(this, msg), msg->value);
}

template <class PointT>
void RadiusOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{

    bool indices_out = msg::isConnected(output_indices_);
    bool cloud_out   = msg::isConnected(output_cloud_);

    if(!indices_out && !cloud_out)
        return;

    int    min_neighbours_= readParameter<int>("min neighbours");
    bool   keep_organized_= readParameter<bool>("keep organized");
    bool   negative_      = readParameter<bool>("negate");
    double search_radius_ = readParameter<double>("search radius");

    typename pcl::PointCloud<PointT>::Ptr cloud_wo_nan(new pcl::PointCloud<PointT>);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud<PointT>(*cloud, *cloud_wo_nan, nan_indices);

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_wo_nan);
    ror.setKeepOrganized(keep_organized_);
    ror.setNegative(negative_);
    ror.setRadiusSearch(search_radius_);
    ror.setMinNeighborsInRadius (min_neighbours_);
    if(msg::hasMessage(indices_input_)) {
        PointIndicesMessage::ConstPtr indices(msg::getMessage<PointIndicesMessage>(indices_input_));
        ror.setIndices(indices->value);
        if(indices->value->indices.size() == 0)
            std::cout << "got empty" << std::endl;
    }
    if(cloud_out) {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        ror.filter(*cloud_filtered);
        PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        cloud_filtered->header = cloud->header;
        out->value = cloud_filtered;
        msg::publish(output_cloud_, out);
    }
    if(indices_out) {
        PointIndicesMessage::Ptr indices_filtered(new PointIndicesMessage);
        indices_filtered->value->header = cloud->header;
        ror.filter(indices_filtered->value->indices);
        msg::publish(output_indices_, indices_filtered);
    }
}
