#pragma once

#include "../data/cluster_data.hpp"
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

namespace csapex
{

template<bool IsDynamic>
class ClusterStorageOps
{
public:
    template<typename PointT, typename Storage>
    static void init(const pcl::PointCloud<PointT>& cloud,
                     const pcl::PointIndices::ConstPtr& indices,
                     const ClusterIndex& indexer,
                     Storage& storage)
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

                                 if (data.state == ClusterDataState::ACCEPTED)
                                 {
                                     auto& indices = accepted[data.cluster].indices;
                                     indices.insert(indices.end(), data.indices.begin(), data.indices.end());
                                 }
                                 else if (data.state == ClusterDataState::REJECTED)
                                 {
                                     auto& indices = rejected[data.cluster].indices;
                                     indices.insert(indices.end(), data.indices.begin(), data.indices.end());
                                 }
                             });
        }
    }
};

}
