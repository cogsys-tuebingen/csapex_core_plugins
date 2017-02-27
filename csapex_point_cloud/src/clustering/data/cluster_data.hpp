#pragma once

#include "cluster_index.hpp"
#include "../../math/mean.hpp"
#include "../../math/distribution.hpp"
#include <cslibs_indexed_storage/storage/auto_index_storage.hpp>
#include <vector>
#include <boost/variant.hpp>
#include <pcl/point_types.h>

namespace csapex
{

namespace detail
{

template<std::size_t I, std::size_t N, typename FeatureList>
struct FeatureOpImpl
{
    template<typename PointT>
    static constexpr void create(FeatureList& self, const PointT& point)
    {
        std::get<I>(self).create(point);
        FeatureOpImpl<I + 1, N, FeatureList>::create(self, point);
    }

    static constexpr void merge(FeatureList& self, const FeatureList& other)
    {
        std::get<I>(self).merge(std::get<I>(other));
        FeatureOpImpl<I + 1, N, FeatureList>::merge(self, other);
    }
};

template<std::size_t N, typename FeatureList>
struct FeatureOpImpl<N, N, FeatureList>
{
    template<typename PointT>
    static constexpr void create(FeatureList&, const PointT&) {}
    static constexpr void merge(FeatureList&, const FeatureList&) {}
};

template<std::size_t I, typename T, typename Tuple>
struct tuple_index_impl;

template<std::size_t I, typename T, typename First, typename... Rest>
struct tuple_index_impl<I, T, std::tuple<First, Rest...>> :
        std::conditional<
                std::is_same<T, First>::value,
                std::integral_constant<std::size_t, I>,
                typename tuple_index_impl<I + 1, T, std::tuple<Rest...>>::type
        >::type
{};

template<std::size_t I, typename T>
struct tuple_index_impl<I, T, std::tuple<>> : std::integral_constant<std::size_t, -1>
{};

template<typename Tuple, typename T>
struct tuple_index : tuple_index_impl<0, T, Tuple>
{};

template<typename Tuple, typename T>
struct tuple_contains :
        std::integral_constant<
                bool,
                tuple_index<Tuple, T>::value != std::size_t(-1)
        >
{};

template<typename Tuple> struct FeatureOp;
template<typename... Features>
struct FeatureOp<std::tuple<Features...>>
{
    using FeatureList = std::tuple<Features...>;
    static constexpr auto Size = sizeof...(Features);

    template<typename PointT>
    static constexpr void create(FeatureList& self, const PointT& point)
    {
        FeatureOpImpl<0, Size, FeatureList>::create(self, point);
    }

    static constexpr void merge(FeatureList& self, const FeatureList& other)
    {
        FeatureOpImpl<0, Size, FeatureList>::merge(self, other);
    }

    template<typename T>
    static constexpr const T& get(const FeatureList& self)
    {
        return std::get<tuple_index<FeatureList, T>::value>(self);
    }
};
}

struct ClusterFeatureDistribution
{
    math::Distribution<3> distribution;

    template<typename PointT>
    inline void create(const PointT& point)
    {
        distribution.add({point.x, point.y, point.z});
    }

    inline void merge(const ClusterFeatureDistribution& other)
    {
        distribution += other.distribution;
    }
};

struct ClusterFeatureColor
{
    boost::variant<math::Mean<1>, math::Mean<3>> color;

    inline void create(...)
    {
    }

    inline void create(const pcl::PointXYZI& point)
    {
        math::Mean<1> mean;
        mean.add(point.intensity);
        color = std::move(mean);
    }

    inline void create(const pcl::PointXYZRGB& point)
    {
        math::Mean<3> mean;
        mean.add({ static_cast<double>(point.r), static_cast<double>(point.g), static_cast<double>(point.b)});
        color = std::move(mean);
    }

    inline void merge(const ClusterFeatureColor& other)
    {
        static UpdateMean updater;
        boost::apply_visitor(updater, color, other.color);
    }

    using DifferenceFunction = double(*)(const Eigen::Vector3d&, const Eigen::Vector3d&, const std::array<double, 3>&);
    inline double difference(DifferenceFunction diff, const std::array<double, 3>& weights, const ClusterFeatureColor& other) const
    {
        return boost::apply_visitor(Difference(diff, weights), color, other.color);
    }

private:
    struct UpdateMean : boost::static_visitor<void>
    {
        template<std::size_t N>
        constexpr void operator()(math::Mean<N>& self, const math::Mean<N>& other) const
        {
            self += other;
        }

        template<std::size_t N1, std::size_t N2, typename = typename std::enable_if<N1 != N2>::type>
        constexpr void operator()(math::Mean<N1>& self, const math::Mean<N2>& other) const
        {}
    };

    struct Difference : boost::static_visitor<double>
    {
        Difference(const DifferenceFunction& diff, const std::array<double, 3>& weights) : diff(diff), weights(weights) {}
        const DifferenceFunction& diff;
        const std::array<double, 3>& weights;

        double operator()(const math::Mean<1>& self, const math::Mean<1>& other) const
        {
            return std::abs(self.getMean() - other.getMean());
        }
        double operator()(const math::Mean<3>& self, const math::Mean<3>& other) const
        {
            return diff(self.getMean(), other.getMean(), weights);
        }

        template<std::size_t N1, std::size_t N2, typename = typename std::enable_if<N1 != N2>::type>
        double operator()(const math::Mean<N1>& self, const math::Mean<N2>& other) const
        {
            return 0.0;
        }
    };
};

enum class ClusterDataState
{
    UNDECIDED,
    ACCEPTED,
    REJECTED
};

template<typename... Features>
struct ClusterData
{
    using FeatureList = std::tuple<Features...>;

    int                     cluster = -1;
    ClusterDataState        state = ClusterDataState::UNDECIDED;
    ClusterIndex::Type      index;
    std::vector<int>        indices;
    FeatureList             features;

    ClusterData() = default;

    template<typename PointT>
    ClusterData(const PointT& point, const ClusterIndex::Type& index, std::size_t id) :
            index(std::move(index))
    {
        indices.push_back(id);
        detail::FeatureOp<FeatureList>::create(features, point);
    }

    inline void merge(const ClusterData& other)
    {
        indices.insert(indices.end(), other.indices.begin(), other.indices.end());
        detail::FeatureOp<FeatureList>::merge(features, other.features);
    }

    template<typename T>
    const T& getFeature() const
    {
        return detail::FeatureOp<FeatureList>::template get<T>(features);
    }
};

}

namespace cslibs_indexed_storage
{

template<typename... Features>
struct auto_index<csapex::ClusterData<Features...>>
{
    constexpr const csapex::ClusterIndex::Type& index(const csapex::ClusterData<Features...>& data) const
    {
        return data.index;
    }
};

}
