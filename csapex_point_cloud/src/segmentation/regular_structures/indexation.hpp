#ifndef INDEXATION_HPP
#define INDEXATION_HPP

#include <array>
#include <algorithm>

namespace csapex {
using DataIndex           = std::array<int, 3>;

template<typename StructureType>
struct Indexation {
    using BinType = std::array<double, 3>;

    BinType bin_sizes;

    Indexation(const BinType &_bin_sizes) :
        bin_sizes(_bin_sizes)
    {
    }

    inline static typename StructureType::Size size(const DataIndex &min_index,
                                                    const DataIndex &max_index)
    {
        typename StructureType::Size size;
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            size[i] = (max_index[i] - min_index[i]) + 1;
        }
        return size;
    }

    template<typename PointT>
    inline constexpr bool is_valid(const PointT& point) const
    {
        bool nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
        bool inf = std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z);
        return !(nan || inf);
    }

    template<typename PointT>
    inline constexpr DataIndex create(const PointT& point) const
    {
        return { static_cast<int>(std::floor(point.x / bin_sizes[0])),
                 static_cast<int>(std::floor(point.y / bin_sizes[1])),
                 static_cast<int>(std::floor(point.z / bin_sizes[2]))};
    }
};


}

#endif // INDEXATION_HPP
