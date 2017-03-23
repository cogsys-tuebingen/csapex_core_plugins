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
    NormalValidatorImpl(const Eigen::Vector3d &normal_final,
                        const double normal_angle_eps) :
        normal_expected_final_(normal_final.normalized()),
        normal_cos_angle_eps_(std::abs(std::cos(normal_angle_eps))),
        normal_track_cluster_normal_(true)
    {
    }

    NormalValidatorImpl(const double normal_angle_eps) :
        normal_cos_angle_eps_(std::abs(std::cos(normal_angle_eps))),
        normal_track_cluster_normal_(false)
    {
    }

    bool start(const Data& data)
    {
        auto& distribution    = data.template getFeature<DistributionFeature>();
        auto& normal          = data.template getFeature<NormalFeature>();
        math::Distribution<3> d = distribution.distribution;
        normal.updateNormal(d);
        cluster_normal_distribution_.reset();
        cluster_normal_distribution_.add(normal.normal);
        return true;
    }

    bool extend(const Data &c, const Data& n)
    {
       auto &neighbor_distribution = n.template getFeature<DistributionFeature>();
       auto &current_normal        = c.template getFeature<NormalFeature>();
       auto &neighbor_normal       = n.template getFeature<NormalFeature>();

       neighbor_normal.update(neighbor_distribution);

       if(normal_track_cluster_normal_) {
            if(compare(cluster_normal_distribution_.getMean(),
                       neighbor_normal.normal)) {
                cluster_normal_distribution_.add(neighbor_normal.normal);
                return true;
            } else {
                return false;
            }
       }

       return compare(current_normal.normal,
                      neighbor_normal.normal);
    }

    bool finish() const
    {
        if(normal_track_cluster_normal_) {
            return compare(cluster_normal_distribution_.getMean(),
                           normal_expected_final_);
        }
        return true;
    }

    inline bool compare(const Eigen::Vector3d &normal_a,
                        const Eigen::Vector3d &normal_b) const
    {
        const double angle = std::abs(normal_a.dot(normal_b));
        if(std::isnan(angle))
            return false;

        return angle >= normal_cos_angle_eps_;
    }

    const Eigen::Vector3d normal_expected_final_;
    const double          normal_cos_angle_eps_;
    const bool            normal_track_cluster_normal_;

    math::Distribution<3> cluster_normal_distribution_;



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
