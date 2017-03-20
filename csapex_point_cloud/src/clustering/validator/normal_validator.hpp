#pragma once

#include "../data/feature_distribution.hpp"
#include "../data/feature_helpers.hpp"
#include "noop_validator.hpp"

#include <type_traits>

namespace csapex { namespace clustering
{

template<typename Data>
struct NormalValidatorImpl
{
public:
    NormalValidatorImpl(bool track_cluster_normal = false)
    {
    }

    bool start(const Data& data)
    {
        auto& feature = data.template getFeature<DistributionFeature>();

        if(track_cluster_normal_) {
            current_distribution_.reset();
            current_distribution_ += feature.distribution;
        }
        return true;
    }

    bool extend(const Data&, const Data& data)
    {
        auto& feature = data.template getFeature<DistributionFeature>();

        /// get normal

        if(track_cluster_normal_) {

        }
        //        current_distribution_ += feature.distribution;

        return true;
    }

    bool finish() const
    {
        if(track_cluster_normal_) {

        }

        /// @todo if check normal then return the final check
        return true;
    }



private:
    inline bool validateCovPCA3D() const
    {
        //        bool valid = true;
        //        math::Distribution<3>::EigenValueSetType eigen_values;
        //        current_distribution_.getEigenValues(eigen_values, true);
        //        /// first sort the eigen values by descending so first paramter always corresponds to
        //        /// the highest value
        //        std::vector<double> eigen_values_vec(eigen_values.data(), eigen_values.data() + 3);
        //        std::sort(eigen_values_vec.begin(), eigen_values_vec.end());

        //        for(std::size_t i = 0 ; i < 3 ; ++i) {
        //            const auto &interval = std_dev_[i];
        //            valid &= eigen_values_vec[i] >= interval.first;
        //            valid &= (interval.second == 0.0 || eigen_values_vec[i] <= interval.second);
        //        }
        //        return valid;
        return true;
    }

private:
    bool                  track_cluster_normal_;
    math::Distribution<3> current_distribution_;

};

template<typename Data>
struct NormalValidator :
        std::conditional<
        detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value,
        NormalValidatorImpl<Data>,
        NoOpValidator<Data>
        >::type
{
    using BaseType = typename std::conditional<
    detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value,
    NormalValidatorImpl<Data>,
    NoOpValidator<Data>
    >::type;

    using BaseType::BaseType;
};


template<typename Data, typename Enable = void>
class Normal {}; // primary template

template<class Data>
class Normal<Data, typename std::enable_if<detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value>::type>
{
}; // specialization for floating point types



}}
