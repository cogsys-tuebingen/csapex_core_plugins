#pragma once

#include "../data/feature_distribution.hpp"
#include "../data/feature_helpers.hpp"
#include "../data/feature_normal.hpp"
#include "noop_validator.hpp"
#include <csapex_point_cloud/math/mean.hpp>

#include <type_traits>

namespace csapex
{
namespace clustering
{
template <typename Data>
struct EIGEN_ALIGN16 NormalValidatorImpl
{
public:
    NormalValidatorImpl(const Eigen::Vector3d& normal_final, const double normal_angle_eps, const double normal_final_angle_eps)
      : normal_expected_final_(normal_final.normalized())
      , normal_cos_angle_eps_(std::abs(std::cos(normal_angle_eps)))
      , normal_cos_angle_eps_final_(std::abs(std::cos(normal_final_angle_eps)))
      , normal_track_cluster_normal_(normal_final_angle_eps != 0.0 && normal_final_angle_eps != M_PI)
      , normal_neighbour_to_neighbour_(normal_angle_eps != 0.0 && normal_angle_eps != M_PI)
    {
    }

    bool start(const Data& data)
    {
        auto& distribution = data.template getFeature<DistributionFeature>();
        auto& normal = data.template getFeature<NormalFeature>();
        math::Distribution<3> d = distribution.distribution;
        normal.updateNormal(d);
        cluster_normal_distribution_.reset();
        cluster_normal_distribution_.add(normal.normal);
        return true;
    }

    bool extend(const Data& c, const Data& n)
    {
        auto& neighbor_distribution = n.template getFeature<DistributionFeature>();
        auto& current_normal = c.template getFeature<NormalFeature>();
        auto& neighbor_normal = n.template getFeature<NormalFeature>();

        math::Distribution<3> d = neighbor_distribution.distribution;
        neighbor_normal.updateNormal(d);

        if (normal_track_cluster_normal_) {
            if (compare(cluster_normal_distribution_.getMean(), neighbor_normal.normal)) {
                cluster_normal_distribution_.add(neighbor_normal.normal);
                return true;
            } else {
                return false;
            }
        }

        if (normal_neighbour_to_neighbour_)
            return compare(current_normal.normal, neighbor_normal.normal);
        return true;
    }

    bool finish() const
    {
        if (normal_track_cluster_normal_) {
            return compare(cluster_normal_distribution_.getMean());
        }
        return true;
    }

    inline bool compare(const Eigen::Vector3d& normal_a, const Eigen::Vector3d& normal_b) const
    {
        const double angle = std::abs(normal_a.dot(normal_b));
        if (std::isnan(angle))
            return false;

        return angle >= normal_cos_angle_eps_;
    }

    inline bool compare(const Eigen::Vector3d& normal) const
    {
        const double angle = std::abs(normal_expected_final_.dot(normal));
        if (std::isnan(angle))
            return false;

        return angle >= normal_cos_angle_eps_final_;
    }

    const Eigen::Vector3d normal_expected_final_;
    const double normal_cos_angle_eps_;
    const double normal_cos_angle_eps_final_;
    bool normal_track_cluster_normal_;
    bool normal_neighbour_to_neighbour_;

    math::Distribution<3> cluster_normal_distribution_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename Data>
struct NormalValidator : std::conditional<detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value && detail::tuple_contains<typename Data::FeatureList, NormalFeature>::value,
                                          NormalValidatorImpl<Data>, NoOpValidator<Data>>::type
{
    using BaseType =
        typename std::conditional<detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value && detail::tuple_contains<typename Data::FeatureList, NormalFeature>::value,
                                  NormalValidatorImpl<Data>, NoOpValidator<Data>>::type;

    using BaseType::BaseType;
};
}  // namespace clustering
}  // namespace csapex
