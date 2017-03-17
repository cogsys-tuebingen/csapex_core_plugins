#include <csapex_sample_consensus/models/sac_model_parallel_normal_plane.hpp>

namespace csapex_sample_consensus {
namespace models {
template<>
class ParallelNormalPlane<pcl::PointXYZ, pcl::Normal>;
template<>
class ParallelNormalPlane<pcl::PointXYZI, pcl::Normal>;
template<>
class ParallelNormalPlane<pcl::PointXYZRGB, pcl::Normal>;
template<>
class ParallelNormalPlane<pcl::PointXYZRGBA, pcl::Normal>;
template<>
class ParallelNormalPlane<pcl::PointXYZRGBL, pcl::Normal>;
template<>
class ParallelNormalPlane<pcl::PointXYZL, pcl::Normal>;
}
}
