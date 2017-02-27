#pragma once

#include <array>

namespace csapex
{

class ClusterIndex
{
public:
    using Type = std::array<int, 3>;

    constexpr ClusterIndex(float size_x, float size_y, float size_z) :
            voxel_size_{size_x, size_y, size_z}
    {}

    template<typename PointT>
    constexpr Type createIndex(const PointT& point) const
    {
        return {{ static_cast<int>(point.x / voxel_size_[0]),
                        static_cast<int>(point.y / voxel_size_[1]),
                        static_cast<int>(point.z / voxel_size_[2]) }};
    }

    template<typename PointT>
    constexpr bool isValid(const PointT& point) const
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

private:
    std::array<float, 3> voxel_size_;
};

}
