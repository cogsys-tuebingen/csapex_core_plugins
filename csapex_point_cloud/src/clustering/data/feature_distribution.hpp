#pragma once

#include <csapex_point_cloud/math/distribution.hpp>

namespace csapex { namespace clustering
{

struct DistributionFeature
{
    math::Distribution<3> distribution;

    template<typename PointT>
    inline void create(const PointT& point)
    {
        distribution.add({ point.x, point.y, point.z });
    }

    inline void merge(const DistributionFeature& other)
    {
        distribution += other.distribution;
    }
};

}}
