#pragma once

#include <csapex/model/node.h>
#include <pcl/point_cloud.h>

#include "validator/distribution_validator.hpp"
#include "validator/color_validator.hpp"

namespace csapex { namespace clustering {

class ClusterPointCloud : public csapex::Node
{
public:
    enum class BackendType { PAGED, KDTREE, ARRAY };

    void setupParameters(Parameterizable& parameters) override;
    void setup(NodeModifier& node_modifier) override;
    void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    template<typename PointT>
    void selectData(typename pcl::PointCloud<PointT>::ConstPtr cloud);
    template<typename DataType, typename PointT>
    void selectStorage(typename pcl::PointCloud<PointT>::ConstPtr cloud);
    template<typename Storage, typename PointT>
    void clusterCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input*  in_pointcloud_;
    Input*  in_indices_;
    Output* out_clusters_accepted_;
    Output* out_clusters_rejected_;

    BackendType backend_;

    std::array<double, 3> voxel_size_;
    std::pair<int, int> cluster_point_count_;

    bool voxel_validation_enabled_;
    int voxel_validation_min_count_;
    double voxel_validation_scale_;

    bool distribution_enabled_;
    DistributionAnalysisType distribution_type_;
    std::array<std::pair<double, double>, 3> distribution_std_dev_;

    bool color_enabled_;
    ColorDifferenceType color_type_;
    std::array<double, 3> color_weights_;
    double color_threshold_;
};

}}
