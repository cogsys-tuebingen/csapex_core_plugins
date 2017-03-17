#ifndef SAC_MODEL_FROM_NORMALS_HPP
#define SAC_MODEL_FROM_NORMALS_HPP

/// PROJECT
#include "sac_model_from_normals.h"

namespace csapex_sample_consensus {
namespace models {
template<typename PointT, typename NormalT>
ModelFromNormals<PointT, NormalT>::ModelFromNormals
    (const typename PointCloud::ConstPtr  &pointcloud,
     const typename NormalCloud::ConstPtr &normalcloud,
     const float normal_distance_weight) :
 ModelFromNormals<PointT, NormalT>::Base(pointcloud),
 normalcloud_(normalcloud),
 normal_distance_weight_(normal_distance_weight)
{
}
}
}

#endif // SAC_MODEL_FROM_NORMALS_HPP
