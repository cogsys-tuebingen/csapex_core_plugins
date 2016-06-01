#ifndef CLUSTER_POINTCLOUD_H
#define CLUSTER_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <pcl/PointIndices.h>

namespace csapex {
class ClusterPointcloud : public csapex::Node
{
public:
    ClusterPointcloud();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices> >
    pclEuclidean(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices> >
    polar(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    enum class Method {
        PCL_EUCLIDEAN,
        POLAR
    };

private:
    Input* in_cloud_;
    Input* in_indices_;
    Output* out_;
    Output* out_debug_;

    Method method_;

    double param_clusterTolerance_;
    int param_clusterMinSize_;
    int param_clusterMaxSize_;

    double opening_angle_;
};
}
#endif // CLUSTER_POINTCLOUD_H
