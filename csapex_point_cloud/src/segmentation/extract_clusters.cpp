/// PROJECT
#include <csapex/model/node.h>
#include <csapex/msg/io.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/register_apex_plugin.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/indices_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex_ros/yaml_io.hpp>
#include <csapex_ros/ros_message_conversion.h>
#include <csapex/view/utility/color.hpp>
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/view/utility/color.hpp>

/// SYSTEM
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>

using namespace csapex::connection_types;


namespace csapex
{


namespace impl {

template <class PointT>
struct Impl;

}


class ExtractClusters : public Node
{
public:
    ExtractClusters()
    {

    }

    void setup(csapex::NodeModifier& modifier) override
    {
        in_ = modifier.addInput<PointCloudMessage>("Cloud");
        in_indices_ = modifier.addInput<GenericVectorMessage, pcl::PointIndices>("Indices");

        out_ = modifier.addOutput<GenericVectorMessage, PointCloudMessage::ConstPtr>("Clusters");
    }

    void setupParameters(csapex::Parameterizable& params) override
    {
    }

    void process()
    {
        PointCloudMessage::ConstPtr msg(msg::getMessage<PointCloudMessage>(in_));

        boost::apply_visitor (PointCloudMessage::Dispatch<ExtractClusters>(this, msg), msg->value);
    }

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr)
    {
        const pcl::PointCloud<PointT>& cloud = *cloud_ptr;

        auto cluster_indices = msg::getMessage<GenericVectorMessage, pcl::PointIndices>(in_indices_);
        const std::vector<pcl::PointIndices>& indices = *cluster_indices;

        std::shared_ptr<std::vector<PointCloudMessage::ConstPtr>> clusters = std::make_shared<std::vector<PointCloudMessage::ConstPtr>>();

        for(const pcl::PointIndices& cluster_idx : indices) {

            typename pcl::PointCloud<PointT>::Ptr cluster_pts(new pcl::PointCloud<PointT>);
            cluster_pts->reserve(cluster_idx.indices.size());
            for(int i : cluster_idx.indices) {
                cluster_pts->push_back(cloud.at(i));
            }
            cluster_pts->header = cloud_ptr->header;

            PointCloudMessage::Ptr cluster = std::make_shared<PointCloudMessage>(cloud.header.frame_id, cloud.header.stamp);
            cluster->value = cluster_pts;
            clusters->push_back(cluster);
        }

        msg::publish<GenericVectorMessage, PointCloudMessage::ConstPtr>(out_, clusters);
    }


private:
    Input* in_;
    Input* in_indices_;

    Output* out_;
};


namespace impl {

template <class PointT>
struct Impl
{
    static void inputCloud(ExtractClusters* instance, typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        instance->inputCloud(cloud);
    }
};

}

}

CSAPEX_REGISTER_CLASS(csapex::ExtractClusters, csapex::Node)

