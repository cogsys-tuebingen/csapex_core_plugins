#include <csapex_sample_consensus/models/sac_model_normal_plane.hpp>

#include <pcl/point_types.h>

namespace csapex_sample_consensus
{
namespace models
{
template <>
class NormalPlane<pcl::PointXYZ, pcl::Normal>;
template <>
class NormalPlane<pcl::PointXYZI, pcl::Normal>;
template <>
class NormalPlane<pcl::PointXYZRGB, pcl::Normal>;
template <>
class NormalPlane<pcl::PointXYZRGBA, pcl::Normal>;
template <>
class NormalPlane<pcl::PointXYZRGBL, pcl::Normal>;
template <>
class NormalPlane<pcl::PointXYZL, pcl::Normal>;
}  // namespace models
}  // namespace csapex_sample_consensus
