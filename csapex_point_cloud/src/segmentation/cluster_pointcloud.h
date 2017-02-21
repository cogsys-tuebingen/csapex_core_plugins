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
    pclRegionGrowing(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices> >
    polar(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    enum class Method {
        PCL_EUCLIDEAN,
        PCL_REGION_GROWING,
        POLAR
    };

private:
    Input*  in_cloud_;
    Input*  in_indices_;
    Output* out_;
    Output* out_debug_;

    Method  method_;

    double  param_cluster_tolerance_;
    int     param_cluster_min_size_;
    int     param_cluster_max_size_;

    int     param_k_search_;
    int     param_neighbours_;
    double  param_smoothness_;
    double  param_curvature_threshold_;

    double opening_angle_;
};
}
#endif // CLUSTER_POINTCLOUD_H
