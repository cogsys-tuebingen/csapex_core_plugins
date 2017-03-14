#pragma once

#include <array>

namespace csapex { namespace clustering
{

class VoxelIndex
{
public:
    using Type = std::array<int, 3>;

    constexpr VoxelIndex(float size_x, float size_y, float size_z) :
            voxel_size_{ size_x, size_y, size_z }
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

    static inline void minmax(const Type& index, Type& min, Type& max)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            min[i] = std::min(index[i], min[i]);
            max[i] = std::max(index[i], max[i]);
        }
    }

private:
    std::array<float, 3> voxel_size_;
};

}}
