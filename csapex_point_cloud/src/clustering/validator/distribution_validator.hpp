#pragma once

#include "../data/feature_distribution.hpp"
#include "../data/feature_helpers.hpp"
#include "noop_validator.hpp"

namespace csapex
{
namespace clustering
{
enum class DistributionAnalysisType
{
    DEFAULT,
    PCA2D,
    PCA3D
};

template <typename Data>
struct DistributionValidatorImpl
{
public:
    DistributionValidatorImpl(DistributionAnalysisType type, std::array<std::pair<double, double>, 3> std_dev) : type_(type), std_dev_(std_dev)
    {
        static const auto square = [](double& value) { value *= value; };
        for (std::size_t i = 0; i < 3; ++i) {
            square(std_dev_[i].first);
            square(std_dev_[i].second);
        }
    }

    bool start(const Data& data)
    {
        auto& feature = data.template getFeature<DistributionFeature>();

        current_distribution_.reset();
        current_distribution_ += feature.distribution;

        return true;
    }

    bool extend(const Data&, const Data& data)
    {
        auto& feature = data.template getFeature<DistributionFeature>();

        current_distribution_ += feature.distribution;

        return true;
    }

    bool finish() const
    {
        switch (type_) {
            default:
            case DistributionAnalysisType::DEFAULT:
                return validateCovDefault();
            case DistributionAnalysisType::PCA2D:
                return validateCovPCA2D();
            case DistributionAnalysisType::PCA3D:
                return validateCovPCA3D();
        }
    }

private:
    inline bool validateCovDefault() const
    {
        bool valid = true;
        math::Distribution<3>::MatrixType cov;
        current_distribution_.getCovariance(cov);
        for (std::size_t i = 0; i < 3; ++i) {
            const auto& interval = std_dev_[i];
            valid &= cov(i, i) >= interval.first;
            valid &= (interval.second == 0.0 || cov(i, i) <= interval.second);
        }
        return valid;
    }

    inline bool validateCovPCA2D() const
    {
        bool valid = true;
        math::Distribution<3>::MatrixType cov3D;
        current_distribution_.getCovariance(cov3D);

        Eigen::Matrix2d cov2D = cov3D.block<2, 2>(0, 0);
        Eigen::EigenSolver<Eigen::Matrix2d> solver(cov2D);
        Eigen::Vector2d eigen_values = solver.eigenvalues().real();

        for (std::size_t i = 0; i < 2; ++i) {
            const auto& interval = std_dev_[i];
            valid &= eigen_values[i] >= interval.first;
            valid &= (interval.second == 0.0 || eigen_values[i] <= interval.second);
        }

        return valid;
    }

    inline bool validateCovPCA3D() const
    {
        bool valid = true;
        math::Distribution<3>::EigenValueSetType eigen_values;
        current_distribution_.getEigenValues(eigen_values, true);
        /// first sort the eigen values by descending so first paramter always
        /// corresponds to the highest value
        std::vector<double> eigen_values_vec(eigen_values.data(), eigen_values.data() + 3);
        std::sort(eigen_values_vec.begin(), eigen_values_vec.end());

        for (std::size_t i = 0; i < 3; ++i) {
            const auto& interval = std_dev_[i];
            valid &= eigen_values_vec[i] >= interval.first;
            valid &= (interval.second == 0.0 || eigen_values_vec[i] <= interval.second);
        }
        return valid;
    }

private:
    math::Distribution<3> current_distribution_;

private:
    DistributionAnalysisType type_;
    std::array<std::pair<double, double>, 3> std_dev_;
};

template <typename Data>
struct DistributionValidator : std::conditional<detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value, DistributionValidatorImpl<Data>, NoOpValidator<Data>>::type
{
    using BaseType = typename std::conditional<detail::tuple_contains<typename Data::FeatureList, DistributionFeature>::value, DistributionValidatorImpl<Data>, NoOpValidator<Data>>::type;

    using BaseType::BaseType;
};

}  // namespace clustering
}  // namespace csapex
