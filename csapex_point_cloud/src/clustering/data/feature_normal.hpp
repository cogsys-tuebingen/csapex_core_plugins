#pragma once

#include "feature_distribution.hpp"
#include <csapex_point_cloud/math/plane.hpp>

namespace csapex
{
namespace clustering
{
struct NormalFeature
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    mutable Eigen::Vector3d normal = { 0.0, 0.0, 0.0 };
    mutable Eigen::Vector3d origin = { 0.0, 0.0, 0.0 };
    mutable bool dirty = true;
    mutable bool valid = true;

    template <typename PointT>
    inline void create(const PointT& point)
    {
    }

    inline void merge(const NormalFeature& other)
    {
        dirty = true;
    }

    inline bool updateNormal(math::Distribution<3>& distribution) const
    {
        if (dirty) {
            valid = math::Plane::fit(distribution, origin, normal);
            normal.normalize();
            dirty = false;
        }
        return valid;
    }
};
}  // namespace clustering
}  // namespace csapex
