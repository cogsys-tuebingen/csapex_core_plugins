#pragma once

#include "../data/cluster_index.hpp"
#include "../data/cluster_data.hpp"
#include <cslibs_indexed_storage/operations/clustering.hpp>

namespace csapex
{

namespace detail
{

template<std::size_t I, std::size_t N, typename Tuple>
struct ValidatorVisitorImpl
{
    static constexpr bool finish(Tuple& visitors)
    {
        return std::get<I>(visitors).finish()
               & ValidatorVisitorImpl<I + 1, N, Tuple>::finish(visitors);
    }

    template<typename Data>
    static constexpr bool start(Tuple& visitors, const Data& data)
    {
        return std::get<I>(visitors).start(data)
               & ValidatorVisitorImpl<I + 1, N, Tuple>::start(visitors, data);
    }

    template<typename Data>
    static constexpr bool extend(Tuple& visitors, const Data& data, const Data& other)
    {
        return std::get<I>(visitors).extend(data, other)
               & ValidatorVisitorImpl<I + 1, N, Tuple>::extend(visitors, data, other);
    }
};

template<std::size_t N, typename Tuple>
struct ValidatorVisitorImpl<N, N, Tuple>
{
    static constexpr bool finish(Tuple&) { return true; }

    template<typename Data>
    static constexpr bool start(Tuple&, const Data&) { return true; }

    template<typename Data>
    static constexpr bool extend(Tuple&, const Data&, const Data&) { return true; }
};

template<typename Tuple>
struct ValidatorVisitor
{
    static constexpr auto Size = std::tuple_size<Tuple>::value;

    static constexpr bool finish(Tuple& visitors)
    {
        return ValidatorVisitorImpl<0, Size, Tuple>::finish(visitors);
    }

    template<typename Data>
    static constexpr bool start(Tuple& visitors, const Data& data)
    {
        return ValidatorVisitorImpl<0, Size, Tuple>::start(visitors, data);
    }

    template<typename Data>
    static constexpr bool extend(Tuple& visitors, const Data& data, const Data& other)
    {
        return ValidatorVisitorImpl<0, Size, Tuple>::extend(visitors, data, other);
    }
};

}

template<typename Storage, typename... Validators>
class ClusterOp
{
public:
    using Index = typename Storage::index_t;
    using Data = typename Storage::data_t;
    using ValidatorList = std::tuple<Validators&...>;

    ClusterOp(const Storage& storage, Validators&... validator) :
            storage_(storage),
            validators_(std::forward_as_tuple(validator...))
    {}

    std::size_t getClusterCount() const
    {
        return current_cluster_index_ + 1;
    }

    bool start(const Index&, Data& data)
    {
        if (data.cluster != -1)
            return false;

        if (!current_cluster_.empty())
            commitCluster(detail::ValidatorVisitor<ValidatorList>::finish(validators_));

        if (!detail::ValidatorVisitor<ValidatorList>::start(validators_, data))
            return false;

        current_cluster_index_ +=1;

        data.cluster = current_cluster_index_;
        current_cluster_.push_back(&data);

        return true;
    }

    bool extend(const Index& center, const Index&, Data& data)
    {
        if (data.cluster != -1)
            return false;

        if (!detail::ValidatorVisitor<ValidatorList>::extend(validators_, *(storage_.get(center)), data))
            return false;

        data.cluster = current_cluster_index_;
        current_cluster_.push_back(&data);

        return true;
    }

    // 3x3x3 neighborhood
    using neighborhood_t = cslibs_indexed_storage::operations::clustering::GridNeighborhoodStatic<std::tuple_size<Index>::value, 3>;
    using visitor_index_t = typename neighborhood_t::offset_t;

    //! vistor implementation for neighbors
    template<typename visitor_t>
    void visit_neighbours(const Index&, const visitor_t& visitior)
    {
        static constexpr auto neighborhood = neighborhood_t{};
        neighborhood.visit(visitior);
    }

private:
    void commitCluster(bool accepted)
    {
        for (Data* data : current_cluster_)
            data->state = accepted ? ClusterDataState::ACCEPTED : ClusterDataState::REJECTED;

        current_cluster_.clear();
    }

private:
    int current_cluster_index_ = -1;
    std::vector<Data*> current_cluster_;
    const Storage& storage_;
    ValidatorList validators_;
};

}
