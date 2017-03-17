#ifndef SAC_MODEL_FROM_NORMALS_H
#define SAC_MODEL_FROM_NORMALS_H

/// PROJECT
#include "sac_model.h"

namespace csapex_sample_consensus {
namespace models {
template<typename PointT, typename NormalT>
class ModelFromNormals : public Model<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using NormalCloud = pcl::PointCloud<NormalT>;
    using Base = Model<PointT>;

    ModelFromNormals(const typename PointCloud::ConstPtr  &pointcloud,
                                    const typename NormalCloud::ConstPtr &normalcloud,
                                    const float normal_distance_weight = 0.f);

protected:
    typename NormalCloud::ConstPtr normalcloud_;
    float normal_distance_weight_;

};
}
}

#endif // SAC_MODEL_FROM_NORMALS_H
