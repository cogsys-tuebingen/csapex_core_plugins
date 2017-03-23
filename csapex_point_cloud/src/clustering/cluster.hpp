#pragma once

#include <csapex/model/node.h>
#include <pcl/point_cloud.h>

#include "validator/distribution_validator.hpp"
#include "validator/color_validator.hpp"
#include "validator/normal_validator.hpp"

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
    // I/O ports
    Input*  in_pointcloud_;
    Input*  in_indices_;
    Output* out_clusters_accepted_;
    Output* out_clusters_rejected_;
    Output* out_voxels_;

    // backend config
    BackendType backend_;

    // general clustering config
    std::array<double, 3>   voxel_size_;
    std::pair<int, int>     cluster_point_count_;

    // voxel pre-clustering validation
    bool    voxel_validation_enabled_;
    int     voxel_validation_min_count_;
    double  voxel_validation_scale_;

    // cluster distribution validation
    bool                                        distribution_enabled_;
    DistributionAnalysisType                    distribution_type_;
    std::array<std::pair<double, double>, 3>    distribution_std_dev_;

    /// normal filter
    Eigen::Vector3d                             validation_normal_;
    bool                                        validate_normal_;
    double                                      validation_normal_angle_eps_;

    // cluster color validation
    bool                                        color_enabled_;
    ColorDifferenceType                         color_type_;
    std::array<double, 3>                       color_weights_;
    double                                      color_threshold_;

    /// neighbour normal validation
    bool                                        normal_enabled_;
    bool                                        normal_mean_;
    double                                      normal_angle_eps_;

};

}}
