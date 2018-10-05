#pragma once

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

/// SYSTEM
#include <pcl/PointIndices.h>
#include <pcl/pcl_base.h>

namespace csapex
{
namespace clustering
{
class ClusterPointCloudPCL : public csapex::Node
{
public:
    ClusterPointCloudPCL();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices>> pclEuclidean(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::IndicesConstPtr indices);

    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices>> pclPolar(typename pcl::PointCloud<PointT>::ConstPtr cloud, pcl::IndicesConstPtr indices);

private:
    enum class Method
    {
        PCL_EUCLIDEAN,
        PCL_POLAR
    };

private:
    Input* in_cloud_;
    Input* in_indices_;
    Output* out_;
    Output* out_debug_;

    Method method_;

    double cluster_tolerance_;
    int cluster_min_size_;
    int cluster_max_size_;

    double polar_opening_angle_;
};

}  // namespace clustering
}  // namespace csapex
