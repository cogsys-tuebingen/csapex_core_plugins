#pragma once

#include "voxel_index.hpp"
#include "feature_helpers.hpp"
#include "../../math/mean.hpp"
#include <cslibs_indexed_storage/storage/auto_index_storage.hpp>
#include <vector>

namespace csapex { namespace clustering {

enum class VoxelState
{
    UNDECIDED,
    ACCEPTED,
    REJECTED,
    INVALID,
};

template<typename... Features>
struct VoxelData
{
    using FeatureList = std::tuple<Features...>;

    int                 cluster = -1;
    VoxelState          state = VoxelState::UNDECIDED;
    VoxelIndex::Type    index;
    std::vector<int>    indices;
    FeatureList         features;
    math::Mean<1>       depth;

    VoxelData() = default;

    template<typename PointT>
    VoxelData(const PointT& point, const VoxelIndex::Type& index, std::size_t id) :
            index(std::move(index))
    {
        indices.push_back(id);
        depth.add(std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
        detail::FeatureOp<FeatureList>::create(features, point);
    }

    inline void merge(const VoxelData& other)
    {
        indices.insert(indices.end(), other.indices.begin(), other.indices.end());
        depth += other.depth;
        detail::FeatureOp<FeatureList>::merge(features, other.features);
    }

    template<typename T>
    const T& getFeature() const
    {
        return detail::FeatureOp<FeatureList>::template get<T>(features);
    }
};

}}

namespace cslibs_indexed_storage
{

template<typename... Features>
struct auto_index<csapex::clustering::VoxelData<Features...>>
{
    constexpr const csapex::clustering::VoxelIndex::Type& index(const csapex::clustering::VoxelData<Features...>& data) const
    {
        return data.index;
    }
};

}
