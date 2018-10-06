#ifndef SAC_MODEL_PARALLEL_NORMAL_PLANE_H
#define SAC_MODEL_PARALLEL_NORMAL_PLANE_H

#include "sac_model_normal_plane.h"

namespace csapex_sample_consensus
{
namespace models
{
template <typename PointT, typename NormalT>
class ParallelNormalPlane : public NormalPlane<PointT, NormalT>
{
public:
    using Base = NormalPlane<PointT, NormalT>;
    using PointCloud = typename Base::PointCloud;
    using NormalCloud = typename Base::NormalCloud;

    ParallelNormalPlane(const typename PointCloud::ConstPtr& pointcloud, const typename NormalCloud::ConstPtr& normalcloud, const float normal_distance_weight = 0.f);

    virtual typename Base::Base::Ptr clone() const override;

    virtual bool isValid() const override;

    void setAxis(const NormalT& axis, const float angle_eps);

protected:
    float angle_eps_;
    float angle_eps_cos_;

    NormalT axis_;

    inline float dotAxis() const;
};
}  // namespace models
}  // namespace csapex_sample_consensus

#endif  // SAC_MODEL_PARALLEL_NORMAL_PLANE_H
