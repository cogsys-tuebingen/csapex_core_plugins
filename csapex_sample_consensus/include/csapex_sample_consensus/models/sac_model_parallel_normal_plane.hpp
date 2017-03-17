#ifndef SAC_MODEL_PARALLEL_NORMAL_PLANE_HPP
#define SAC_MODEL_PARALLEL_NORMAL_PLANE_HPP

/// PROJECT
#include "sac_model_parallel_normal_plane.h"

namespace csapex_sample_consensus {
namespace models {
template<typename PointT, typename NormalT>
ParallelNormalPlane<PointT, NormalT>::ParallelNormalPlane(const typename PointCloud::ConstPtr &pointcloud,
                                                                    const typename NormalCloud::ConstPtr &normalcloud,
                                                                    const float normal_distance_weight) :
    Base(pointcloud, normalcloud, normal_distance_weight),
    angle_eps_(0.f)
{
}

template<typename PointT, typename NormalT>
inline typename ParallelNormalPlane<PointT, NormalT>::Base::Base::Ptr ParallelNormalPlane<PointT, NormalT>::clone() const
{
    ParallelNormalPlane<PointT, NormalT> *plane =
            new ParallelNormalPlane<PointT, NormalT>(Base::pointcloud_,
                                                          Base::normalcloud_,
                                                          Base::normal_distance_weight_);
    plane->model_coefficients_ = Base::model_coefficients_;
    plane->model_indices_      = Base::model_indices_;
    plane->setAxis(axis_, angle_eps_);

    return typename Base::Ptr(plane);
}

template<typename PointT, typename NormalT>
inline bool ParallelNormalPlane<PointT, NormalT>::isModelValid() const
{
    if(Base::model_coefficients_.size() != 4)
        return false;

    if(angle_eps_ > 0.f) {
        if(dotAxis() < angle_eps_cos_)
            return false;
    }

    return true;
}

template<typename PointT, typename NormalT>
inline void ParallelNormalPlane<PointT, NormalT>::setAxis(const NormalT &axis,
                                                               const float angle_eps)
{
    axis_ = axis;
    angle_eps_ = angle_eps;
    angle_eps_cos_ = std::cos(angle_eps);
}

template<typename PointT, typename NormalT>
inline float ParallelNormalPlane<PointT, NormalT>::dotAxis() const
{
    return std::abs(Base::model_coefficients_[0] * axis_.x +
                    Base::model_coefficients_[1] * axis_.y +
                    Base::model_coefficients_[2] * axis_.z);
}
}
}

#endif // SAC_MODEL_PARALLEL_NORMAL_PLANE_HPP
