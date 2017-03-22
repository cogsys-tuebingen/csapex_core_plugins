#pragma once

#include <csapex_point_cloud/math/mean.hpp>
#include "../data/feature_normal.hpp"
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
    NormalValidatorImpl(const Eigen::Vector3d &preferred_normal,
                        const double angle_eps) :
        preferred_normal_(preferred_normal.normalized()),
        preferred_normal_cos_angle_eps_(std::abs(std::cos(angle_eps)))
    {
    }

    bool start(const Data& data)
    {
        auto& distribution    = data.template getFeature<DistributionFeature>();
        auto& normal          = data.template getFeature<NormalFeature>();
        current_mean_normal_.reset();

        current_distribution_ = distribution.distribution;
        normal.updateNormal(current_distribution_);
        current_mean_normal_.add(normal.normal);

        return true;
    }

    bool extend(const Data&, const Data& data)
    {
        auto& distribution = data.template getFeature<DistributionFeature>();
        auto& normal       = data.template getFeature<NormalFeature>();
        current_distribution_ += distribution.distribution;
        normal.updateNormal(current_distribution_);
        current_mean_normal_.add(normal.normal);

        return true;
    }

    bool finish() const
    {
        const  Eigen::Vector3d mean = current_mean_normal_.getMean();
        if(std::isnan(mean(0)) ||
                std::isnan(mean(1)) ||
                    std::isnan(mean(2)))
            return false;

        const double angle = std::abs(preferred_normal_.dot(current_mean_normal_.getMean()));
        return angle >= preferred_normal_cos_angle_eps_;
    }

    const Eigen::Vector3d preferred_normal_;
    const double          preferred_normal_cos_angle_eps_;

    math::Distribution<3> current_distribution_;
    math::Mean<3>         current_mean_normal_;


};

template<typename Data>
struct NormalValidator :
        std::conditional<
        detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value &&
        detail::tuple_contains<typename Data::FeatureList, NormalFeature>::value,
        NormalValidatorImpl<Data>,
        NoOpValidator<Data>
        >::type
{
    using BaseType = typename std::conditional<
    detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value &&
    detail::tuple_contains<typename Data::FeatureList, NormalFeature>::value,
    NormalValidatorImpl<Data>,
    NoOpValidator<Data>
    >::type;

    using BaseType::BaseType;
};
}}
