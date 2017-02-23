#ifndef SAC_SEGMENTATION_HPP
#define SAC_SEGMENTATION_HPP

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

namespace csapex {
template<typename PointT>
class SACSegmentation : public pcl::SACSegmentation<PointT> {



};

template<typename PointT, typename NormalT>
class SACSegmentationFromNormals : public pcl::SACSegmentationFromNormals<PointT, NormalT>
{

};
}



#endif // SAC_SEGMENTATION_HPP
