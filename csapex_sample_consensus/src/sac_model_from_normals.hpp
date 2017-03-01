#ifndef SAC_MODEL_FROM_NORMALS_HPP
#define SAC_MODEL_FROM_NORMALS_HPP

#include "sac_model.hpp"

namespace sample_consensus {
template<typename PointT, typename NormalT>
class SampleConsensusModelFromNormals : public SampleConsensusModel<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using NormalCloud = pcl::PointCloud<NormalT>;
    using Base = SampleConsensusModel<PointT>;


    SampleConsensusModelFromNormals(const typename PointCloud::ConstPtr  &pointcloud,
                                    const typename NormalCloud::ConstPtr &normalcloud,
                                    const float normal_distance_weight = 0.f) :
        Base(pointcloud),
        normalcloud_(normalcloud),
        normal_distance_weight_(normal_distance_weight)
    {
    }

protected:
    typename NormalCloud::ConstPtr normalcloud_;
    float normal_distance_weight_;

};
}

#endif // SAC_MODEL_FROM_NORMALS_HPP
