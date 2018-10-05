/// HEADER
#include "radius_outlier_removal.h"

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_vector_message.hpp>

/// SYSTEM
// clang-format off
#include <csapex/utility/suppress_warnings_start.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <csapex/utility/suppress_warnings_end.h>
// clang-format on

CSAPEX_REGISTER_CLASS(csapex::RadiusOutlierRemoval, csapex::Node)

using namespace csapex;
using namespace csapex::connection_types;

RadiusOutlierRemoval::RadiusOutlierRemoval()
{
}

void RadiusOutlierRemoval::setupParameters(Parameterizable& parameters)
{
    parameters.addParameter(csapex::param::factory::declareRange("min neighbours", 1, 1000, 2, 1));
    parameters.addParameter(csapex::param::factory::declareBool("keep organized", false));
    parameters.addParameter(csapex::param::factory::declareBool("negate", false));
    parameters.addParameter(csapex::param::factory::declareRange("search radius", 0.0, 30.0, 0.8, 0.01));
}

void RadiusOutlierRemoval::setup(NodeModifier& node_modifier)
{
    input_cloud_ = node_modifier.addInput<PointCloudMessage>("PointCloud");
    indices_input_ = node_modifier.addOptionalMultiInput<PointIndicesMessage, GenericVectorMessage>("indices");
    output_cloud_ = node_modifier.addOutput<PointCloudMessage>("Pointcloud");
    output_indices_ = node_modifier.addOutput<AnyMessage>("indices");
}

void RadiusOutlierRemoval::process()
{
    PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(input_cloud_));
    boost::apply_visitor(PointCloudMessage::Dispatch<RadiusOutlierRemoval>(this, msg), msg->value);
}

template <class PointT>
void RadiusOutlierRemoval::inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    bool indices_out = msg::isConnected(output_indices_);
    bool cloud_out = msg::isConnected(output_cloud_);
    bool has_index = msg::hasMessage(indices_input_);

    if (!indices_out && !cloud_out)
        return;

    int min_neighbours_ = readParameter<int>("min neighbours");
    bool keep_organized_ = readParameter<bool>("keep organized");
    bool negative_ = readParameter<bool>("negate");
    double search_radius_ = readParameter<double>("search radius");

    typename pcl::PointCloud<PointT>::Ptr cloud_wo_nan(new pcl::PointCloud<PointT>);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud<PointT>(*cloud, *cloud_wo_nan, nan_indices);
    pcl::RadiusOutlierRemoval<PointT> ror;
    //    ror.setInputCloud(cloud);
    ror.setInputCloud(cloud_wo_nan);
    ror.setKeepOrganized(keep_organized_);
    ror.setNegative(negative_);
    ror.setRadiusSearch(search_radius_);
    ror.setMinNeighborsInRadius(min_neighbours_);

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    if (has_index) {
        if (msg::isMessage<PointIndicesMessage>(indices_input_)) {
            PointIndicesMessage::ConstPtr indices = msg::getMessage<PointIndicesMessage>(indices_input_);
            if (indices->value->indices.size() == 0)
                std::cout << "got empty" << std::endl;
            ror.setIndices(indices->value);
            if (cloud_out) {
                ror.filter(*cloud_filtered);
                PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
                cloud_filtered->header = cloud->header;
                out->value = cloud_filtered;
                msg::publish(output_cloud_, out);
            }
            if (indices_out) {
                PointIndicesMessage::Ptr indices_filtered(new PointIndicesMessage);
                indices_filtered->value->header = cloud->header;
                ror.filter(indices_filtered->value->indices);
                msg::publish(output_indices_, indices_filtered);
            }
        } else {
            GenericVectorMessage::ConstPtr message = msg::getMessage<GenericVectorMessage>(indices_input_);
            apex_assert(std::dynamic_pointer_cast<GenericPointerMessage<pcl::PointIndices> const>(message->nestedType()));
            std::shared_ptr<std::vector<pcl::PointIndices>> out_ind(new std::vector<pcl::PointIndices>(message->nestedValueCount()));
            auto it = out_ind->begin();
            for (std::size_t i = 0; i < message->nestedValueCount(); ++i) {
                auto val = std::dynamic_pointer_cast<GenericPointerMessage<pcl::PointIndices> const>(message->nestedValue(i));
                pcl::PointIndices::Ptr id = boost::make_shared<pcl::PointIndices>(*val->value);
                ror.setIndices(id);
                if (cloud_out) {
                    ror.filter(*cloud_filtered);
                }
                if (indices_out) {
                    ror.filter(it->indices);
                    ++it;
                }
            }
            if (cloud_out) {
                PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
                cloud_filtered->header = cloud->header;
                out->value = cloud_filtered;
                msg::publish(output_cloud_, out);
            }
            if (indices_out) {
                msg::publish<GenericVectorMessage, pcl::PointIndices>(output_indices_, out_ind);
            }
        }
    } else {
        ror.filter(*cloud_filtered);
        PointCloudMessage::Ptr out(new PointCloudMessage(cloud->header.frame_id, cloud->header.stamp));
        cloud_filtered->header = cloud->header;
        out->value = cloud_filtered;
        msg::publish(output_cloud_, out);
    }
}
