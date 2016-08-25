#ifndef VALIDATOR_HPP
#define VALIDATOR_HPP

#include "cluster_params.hpp"
#include <pcl/PointIndices.h>
#include <vector>

namespace csapex {
template<typename ParamType>
class Validator
{
public:
    enum class Result { ACCEPTED, TOO_SMALL, REJECTED };

    Validator(const ParamType       &params,
              pcl::PointIndices     &indices,
              math::Distribution<3> &distribution) :
        params(params),
        buffer_indices(indices),
        buffer_distribution(distribution)
    {
        check_statistic = false;
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            double &first = this->params.cluster_std_devs[i].first;
            double &second = this->params.cluster_std_devs[i].second;
            check_statistic |= first != 0;
            check_statistic |= second != 0;
            square(first);
            square(second);
        }
    }

    inline Result validate()
    {
        if (!validateSize(buffer_indices.indices.size()))
            return Result::TOO_SMALL;
        if(check_statistic) {
            switch (params.cluster_cov_thresh_type)
            {
            case ClusterParamsStatistical::DEFAULT:
                return validateCovDefault(buffer_distribution) ? Result::ACCEPTED : Result::REJECTED;
            case ClusterParamsStatistical::PCA2D:
                return validateCovPCA2D(buffer_distribution) ? Result::ACCEPTED : Result::REJECTED;
            case ClusterParamsStatistical::PCA3D:
                return validateCovPCA3D(buffer_distribution) ? Result::ACCEPTED : Result::REJECTED;
            default:
                break;
            }
        }
        return Result::ACCEPTED;
    }

private:
    inline void square(double &value)
    {
        value *= value;
    }

    inline bool validateSize(std::size_t size)
    {
        return size >= static_cast<std::size_t>(params.cluster_sizes[0]) &&
                size <= static_cast<std::size_t>(params.cluster_sizes[1]);
    }

    inline bool validateCovDefault(math::Distribution<3>& distribution)
    {
        bool valid = true;
        math::Distribution<3>::MatrixType cov;
        distribution.getCovariance(cov);
        for(std::size_t i = 0 ; i < 3 ; ++i) {
            const auto &interval = params.cluster_std_devs[i];
            valid &= cov(i,i) >= interval.first;
            valid &= (interval.second == 0.0 || cov(i,i) <= interval.second);
        }
        return valid;
    }

    inline bool validateCovPCA2D(math::Distribution<3>& distribution)
    {
        bool valid = true;
        math::Distribution<3>::MatrixType cov3D;
        distribution.getCovariance(cov3D);

        Eigen::Matrix2d cov2D = cov3D.block<2,2>(0,0);
        Eigen::EigenSolver<Eigen::Matrix2d> solver(cov2D);
        Eigen::Vector2d eigen_values  = solver.eigenvalues().real();

        for(std::size_t i = 0 ; i < 2 ; ++i) {
            const auto &interval = params.cluster_std_devs[i];
            valid &= eigen_values[i] >= interval.first;
            valid &= (interval.second == 0.0 || eigen_values[i] <= interval.second);
        }

        return valid;
    }

    inline bool validateCovPCA3D(math::Distribution<3> &distribution)
    {
        bool valid = true;
        math::Distribution<3>::EigenValueSetType eigen_values;
        distribution.getEigenValues(eigen_values, true);
        /// first sort the eigen values by descending so first paramter always corresponds to
        /// the highest value
        std::vector<double> eigen_values_vec(eigen_values.data(), eigen_values.data() + 3);
        std::sort(eigen_values_vec.begin(), eigen_values_vec.end());

        for(std::size_t i = 0 ; i < 3 ; ++i) {
            const auto &interval = params.cluster_std_devs[i];
            valid &= eigen_values_vec[i] >= interval.first;
            valid &= (interval.second == 0.0 || eigen_values_vec[i] <= interval.second);
        }
        return valid;
    }

public:
    ParamType params;
    bool      check_statistic;
private:
    pcl::PointIndices& buffer_indices;
    math::Distribution<3>& buffer_distribution;
};
}
#endif // VALIDATOR_HPP
