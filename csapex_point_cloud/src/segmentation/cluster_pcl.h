#ifndef CLUSTER_POINTCLOUD_H
#define CLUSTER_POINTCLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

/// SYSTEM
#include <pcl/PointIndices.h>
#include <pcl/pcl_base.h>

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
    pclEuclidean(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                 pcl::IndicesConstPtr indices);

    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices> >
    pclRegionGrowing(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                     pcl::IndicesConstPtr indices);

    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices> >
    pclNormal(typename pcl::PointCloud<PointT>::ConstPtr cloud,
              pcl::IndicesConstPtr indices);

    template <class PointT>
    std::shared_ptr<std::vector<pcl::PointIndices> >
    pclPolar(typename pcl::PointCloud<PointT>::ConstPtr cloud,
             pcl::IndicesConstPtr indices);

private:
    enum class Method {
        PCL_EUCLIDEAN,
        PCL_REGION_GROWING,
        PCL_NORMAL,
        PCL_POLAR
    };

private:
    Input*  in_cloud_;
    Input*  in_indices_;
    Output* out_;
    Output* out_debug_;

    Method  method_;

    double  cluster_tolerance_;
    int     cluster_min_size_;
    int     cluster_max_size_;

    int     normal_estimator_k_nearest_;
    double  normal_estimator_radius_;

    int     region_growing_neighbours_;
    double  region_growing_smoothness_;
    double  region_growing_curvature_threshold_;

    double  normal_large_radius_;
    double  normal_small_radius_;


    double  polar_opening_angle_;


};
}
#endif // CLUSTER_POINTCLOUD_H
