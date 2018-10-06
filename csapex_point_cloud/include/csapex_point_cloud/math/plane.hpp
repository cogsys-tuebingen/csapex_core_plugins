#ifndef PLANE_HPP
#define PLANE_HPP

#include "distribution.hpp"

namespace csapex
{
namespace math
{
struct Plane
{
    using DistributionType = Distribution<3>;
    using PointType = DistributionType::PointType;
    using MatrixType = DistributionType::MatrixType;

    static inline bool fit(const DistributionType& distribution, PointType& x_0, PointType& normal)
    {
        if (distribution.getN() <= 2)
            return false;

        const MatrixType covariance = distribution.getCovariance();
        const double determinant = covariance(0, 0) * covariance(1, 1) - covariance(0, 1) * covariance(0, 1);

        if (determinant == 0.0)
            return false;

        x_0 = distribution.getMean();

        /// Cramer
        normal(0) = (covariance(1, 1) * covariance(0, 2) - covariance(0, 1) * covariance(1, 2)) / determinant;  /// A
        normal(1) = (covariance(0, 0) * covariance(0, 2) - covariance(0, 1) * covariance(0, 2)) / determinant;  /// B
        normal(2) = x_0(2) - normal(0) * x_0(0) - normal(1) * x_0(1);                                           /// C
        /// for x_0 : d = 0.0 => C

        return true;
    }
};
}  // namespace math
}  // namespace csapex

#endif  // PLANE_HPP
