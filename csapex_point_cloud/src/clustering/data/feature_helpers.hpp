#pragma once

#include <tuple>
#include <limits>
#include <type_traits>

namespace csapex { namespace clustering { namespace detail
{

template<std::size_t I, std::size_t N, typename FeatureList>
struct FeatureOpImpl
{
    template<typename PointT>
    static void create(FeatureList& self, const PointT& point)
    {
        std::get<I>(self).create(point);
        FeatureOpImpl<I + 1, N, FeatureList>::create(self, point);
    }

    static void merge(FeatureList& self, const FeatureList& other)
    {
        std::get<I>(self).merge(std::get<I>(other));
        FeatureOpImpl<I + 1, N, FeatureList>::merge(self, other);
    }
};

template<std::size_t N, typename FeatureList>
struct FeatureOpImpl<N, N, FeatureList>
{
    template<typename PointT>
    static void create(FeatureList&, const PointT&)
    {}

    static void merge(FeatureList&, const FeatureList&)
    {}
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
{
};

template<std::size_t I, typename T>
struct tuple_index_impl<I, T, std::tuple<>> : std::integral_constant<std::size_t, std::numeric_limits<std::size_t>::max()>
{
};

template<typename Tuple, typename T>
struct tuple_index : tuple_index_impl<0, T, Tuple>
{
};

template<typename Tuple, typename T>
struct tuple_contains :
        std::integral_constant<
                bool,
                tuple_index<Tuple, T>::value != std::size_t(std::numeric_limits<std::size_t>::max())
        >
{
};

template<typename Tuple>
struct FeatureOp;

template<typename... Features>
struct FeatureOp<std::tuple<Features...>>
{
    using FeatureList = std::tuple<Features...>;
    static constexpr auto Size = sizeof...(Features);

    template<typename PointT>
    static void create(FeatureList& self, const PointT& point)
    {
        FeatureOpImpl<0, Size, FeatureList>::create(self, point);
    }

    static void merge(FeatureList& self, const FeatureList& other)
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
}}
