#include <csapex_sample_consensus/models/sac_model_plane.hpp>

namespace csapex_sample_consensus {
namespace models {
template<>
class Plane<pcl::PointXYZ>;
template<>
class Plane<pcl::PointXYZI>;
template<>
class Plane<pcl::PointXYZRGB>;
template<>
class Plane<pcl::PointXYZRGBA>;
template<>
class Plane<pcl::PointXYZRGBL>;
template<>
class Plane<pcl::PointXYZL>;
}
}
