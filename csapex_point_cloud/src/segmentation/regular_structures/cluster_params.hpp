#ifndef CLUSTER_PARAMS_HPP
#define CLUSTER_PARAMS_HPP

#include <distribution.hpp>

namespace csapex {
struct ClusterParams {
    std::array<double, 3> bin_sizes;
    std::array<int, 2>    cluster_sizes;
};

struct ClusterParamsStatistical : public ClusterParams
{
    enum CovarianceThresholdType {DEFAULT, PCA2D, PCA3D};

    std::array<std::pair<double, double>, 3> cluster_std_devs;
    std::array<double, 4>                    cluster_distance_and_weights;
    CovarianceThresholdType                  cluster_cov_thresh_type;

    bool (*validateCovariance)(math::Distribution<3> &distribution,
                               std::array<std::pair<double, double>, 3> &intervals);
};

struct ClusterParamsStatisticalIC  : public ClusterParamsStatistical
{
    enum ColorDifferenceType {CIE94, CIEDE2000, CMC, CIE76};
    ColorDifferenceType color_diff_type;
    double max_ic_difference;
};
}

#endif // CLUSTER_PARAMS_HPP
