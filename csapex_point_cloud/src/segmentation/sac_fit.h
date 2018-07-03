#ifndef SAC_FIT_H
#define SAC_FIT_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>
#include <csapex_point_cloud/msg/model_message.h>

/// POINT CLOUD
#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif //__clang__
#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#if __clang__
#pragma clang diagnostic pop
#endif //__clang__

namespace csapex {
class SacFit : public csapex::Node
{
public:
    SacFit();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    template <class PointT>
    void inputCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud);

private:
    Input*  in_cloud_;
    Input*  in_indices_;
    Output* out_models_;
    Output* out_indices_;

    // PCL parameter
    int     max_iterations_;
    int     min_inliers_;
    double  normal_distance_weight_;
    double  distance_threshold_;
    double  sphere_r_min_;
    double  sphere_r_max_;

    double  model_main_axis_x_;
    double  model_main_axis_y_;
    double  model_main_axis_z_;
    double  model_angle_offset_;

    int     ransac_type_;
    int     model_type_;
    bool    from_normals_;
    bool    optimize_coefficients_;

    template <class PointT>
    void estimateNormals(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr normals);

    bool need_normals();
};
}
#endif // SAC_FIT_H

