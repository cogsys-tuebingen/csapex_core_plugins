/// HEADER
#include "radius_outlier_removal.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_point_cloud/indeces_message.h>
#include <utils_param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>

/// SYSTEM
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

CSAPEX_REGISTER_CLASS(csapex::RadiusOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

RadiusOutlierRemoval::RadiusOutlierRemoval()
{
    addTag(Tag::get("PointCloud"));

    addParameter(param::ParameterFactory::declareRange("min neighbours", 1, 1000, 2, 1));
    addParameter(param::ParameterFactory::declareBool ("keep organized", false));
    addParameter(param::ParameterFactory::declareBool ("negate", false));
    addParameter(param::ParameterFactory::declareRange("search radius", 0.0, 30.0, 0.8, 0.1));
}

void RadiusOutlierRemoval::setup()
{
    input_cloud_ = modifier_->addInput<PointCloudMessage>("PointCloud");
    indeces_input_ = modifier_->addInput<PointIndecesMessage>("Indeces", true);
    output_cloud_ = modifier_->addOutput<PointCloudMessage>("Pointcloud");
    output_indeces_ = modifier_->addOutput<PointIndecesMessage>("Indeces");
}

void RadiusOutlierRemoval::process()
{
    PointCloudMessage::Ptr msg(input_cloud_->getMessage<PointCloudMessage>());
    boost::apply_visitor (PointCloudMessage::Dispatch<RadiusOutlierRemoval>(this, msg), msg->value);
}

template <class PointT>
void RadiusOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{

    bool indeces_out = output_indeces_->isConnected();
    bool cloud_out   = output_cloud_->isConnected();

    if(!indeces_out && !cloud_out)
        return;

    int    min_neighbours_= readParameter<int>("min neighbours");
    bool   keep_organized_= readParameter<bool>("keep organized");
    bool   negative_      = readParameter<bool>("negate");
    double search_radius_ = readParameter<double>("search radius");

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setKeepOrganized(keep_organized_);
    ror.setNegative(negative_);
    ror.setRadiusSearch(search_radius_);
    ror.setMinNeighborsInRadius (min_neighbours_);
    if(indeces_input_->isConnected()) {
        PointIndecesMessage::Ptr indeces(indeces_input_->getMessage<PointIndecesMessage>());
        ror.setIndices(indeces->value);
        if(indeces->value->indices.size() == 0)
            std::cout << "got empty" << std::endl;
    }
    if(cloud_out) {
        typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        ror.filter(*cloud_filtered);
        PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id));
        out->value = cloud_filtered;
        output_cloud_->publish(out);
    }
    if(indeces_out) {
        PointIndecesMessage::Ptr indeces_filtered(new PointIndecesMessage);
        indeces_filtered->value->header = cloud->header;
        ror.filter(indeces_filtered->value->indices);
        output_indeces_->publish(indeces_filtered);
    }
}
