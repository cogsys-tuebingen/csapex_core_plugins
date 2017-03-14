#pragma once

#include "../data/voxel_data.hpp"
#include <cslibs_indexed_storage/backend/array/array_options.hpp>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

namespace csapex { namespace clustering
{

class StorageOperation
{
public:
    // dynamic sized case -> simply add every point to the storage
    template<typename PointT, typename Storage>
    static void init(const pcl::PointCloud<PointT>& cloud,
                     const pcl::PointIndices::ConstPtr& indices,
                     const VoxelIndex& indexer,
                     Storage& storage,
                     std::vector<typename Storage::data_t>&,
                     std::false_type /*not_fixed_size*/)
    {
        const auto add_point = [&storage, &cloud, &indexer](std::size_t id)
        {
            const PointT& point = cloud.at(id);
            if (indexer.isValid(point))
                storage.insert(point, indexer.createIndex(point), id);
        };

        if (indices)
        {
            for (auto id : indices->indices)
                add_point(id);
        }
        else
        {
            for (std::size_t id = 0; id < cloud.size(); ++id)
                add_point(id);
        }
    }

    // fixed size case
    // - create each point in the data_storage
    // - calculate size
    // - initialize storage to size
    // - add non_owning references to storage
    template<typename PointT, typename Storage>
    static void init(const pcl::PointCloud<PointT>& cloud,
                     const pcl::PointIndices::ConstPtr& indices,
                     const VoxelIndex& indexer,
                     Storage& storage,
                     std::vector<typename Storage::data_t>& data_storage,
                     std::true_type /*fixed_size*/)
    {
        const auto add_point = [&data_storage, &cloud, &indexer](std::size_t id)
        {
            const PointT& point = cloud.at(id);
            if (indexer.isValid(point))
                data_storage.emplace_back(point, indexer.createIndex(point), id);
        };

        if (indices)
        {
            data_storage.reserve(indices->indices.size());
            for (auto id : indices->indices)
                add_point(id);
        }
        else
        {
            data_storage.reserve(cloud.size());
            for (std::size_t id = 0; id < cloud.size(); ++id)
                add_point(id);
        }

        VoxelIndex::Type min;
        min.fill(std::numeric_limits<int>::max());
        VoxelIndex::Type max;
        max.fill(std::numeric_limits<int>::min());

        for (const auto& data : data_storage)
            VoxelIndex::minmax(data.index, min, max);

        VoxelIndex::Type size;
        for (std::size_t i = 0; i < 3; ++i)
            size[i] = max[i] - min[i] + 1;

        storage.template set<cslibs_indexed_storage::option::tags::array_offset>(min[0], min[1], min[2]);
        storage.template set<cslibs_indexed_storage::option::tags::array_size>(static_cast<std::size_t>(size[0]),
                                                                               static_cast<std::size_t>(size[1]),
                                                                               static_cast<std::size_t>(size[2]));

        for (auto& data : data_storage)
            storage.insert(&data);
    }

    template<typename Storage, typename ClusterOp>
    static void extract(const Storage& storage,
                        const ClusterOp& cluster_op,
                        std::vector<pcl::PointIndices>& accepted,
                        std::vector<pcl::PointIndices>& rejected)
    {
        using Data = typename Storage::data_t;
        using Index = typename Storage::index_t;

        if (auto count = cluster_op.getClusterCount())
        {
            accepted.resize(count);
            rejected.resize(count);
            storage.traverse([&accepted, &rejected](const Index& index, const Data& data)
                             {
                                 if (data.cluster < 0)
                                     return;

                                 if (data.state == VoxelState::ACCEPTED)
                                 {
                                     auto& indices = accepted[data.cluster].indices;
                                     indices.insert(indices.end(), data.indices.begin(), data.indices.end());
                                 }
                                 else if (data.state == VoxelState::REJECTED)
                                 {
                                     auto& indices = rejected[data.cluster].indices;
                                     indices.insert(indices.end(), data.indices.begin(), data.indices.end());
                                 }
                             });
        }
    }
};

}}
