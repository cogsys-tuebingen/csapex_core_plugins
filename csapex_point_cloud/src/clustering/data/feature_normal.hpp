#pragma once

#include "feature_distribution.hpp"
#include "../../math/plane.hpp"

namespace csapex { namespace clustering {
struct NormalFeature {

    mutable Eigen::Vector3d normal = {0.0, 0.0, 0.0};
    mutable Eigen::Vector3d origin = {0.0, 0.0, 0.0};
    mutable bool            dirty = true;
    mutable bool            valid = true;

    template<typename PointT>
    inline void create(const PointT &point)
    {
    }

    inline void merge(const NormalFeature &other)
    {
        dirty = true;
    }

    inline bool updateNormal(math::Distribution<3> &distribution) const
    {
        if(dirty) {
            valid = math::Plane::fit(distribution,
                                     origin,
                                     normal);
            dirty = false;
        }
        return valid;
    }
};
}}
