#ifndef COLOR_DIFFERENCES_HPP
#define COLOR_DIFFERENCES_HPP

#include <eigen3/Eigen/Core>

namespace csapex {
namespace color_differences {
inline double CIE76 (const Eigen::Vector3d &a,
                     const Eigen::Vector3d &b)
{
    /// for lab
    /// delta_e > 2.3 noticeable difference
    return (a-b).norm();
}

inline double CIE94Grahpics (const Eigen::Vector3d &color_1,
                             const Eigen::Vector3d &color_2)
{
    const double dl = color_1(0) - color_2(0);
    const double C1 = hypot(color_1(1), color_1(2));
    const double C2 = hypot(color_2(1), color_2(2));
    const double dC = C1 - C2;
    const double da = color_1(1) - color_2(1);
    const double db = color_1(2) - color_2(2);
    const double dH = fmax(0.0, da*da + db*db - dC*dC); /// removed sqrt
    const static double Kl = 1;
    const static double K1 = 0.045;
    const static double K2 = 0.015;
    const double s1 = dl/Kl;
    const double s2 = dC/(1. + K1 * C1);
    const double s3 = dH/((1. + K2 * C1)*(1. + K2 * C1));
    return sqrt(s1*s1 + s2*s2 + s3);

}

inline double CIE94Textiles (const Eigen::Vector3d &color_1,
                             const Eigen::Vector3d &color_2)
{
    const double dl = color_1(0) - color_2(0);
    const double C1 = hypot(color_1(1), color_1(2));
    const double C2 = hypot(color_2(1), color_2(2));
    const double dC = C1 - C2;
    const double da = color_1(1) - color_2(1);
    const double db = color_1(2) - color_2(2);
    const double dH = fmax(0.0, da*da + db*db - dC*dC); /// removed sqrt
    const static double Kl = 1;
    const static double K1 = 0.048;
    const static double K2 = 0.014;
    const double s1 = dl/Kl;
    const double s2 = dC/(1. + K1 * C1);
    const double s3 = dH/((1. + K2 * C1)*(1. + K2 * C1));
    return sqrt(s1*s1 + s2*s2 + s3);

}
}
}
#endif // COLOR_DIFFERENCES_HPP
