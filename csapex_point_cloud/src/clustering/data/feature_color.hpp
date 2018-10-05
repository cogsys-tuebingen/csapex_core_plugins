#pragma once

#include <boost/variant.hpp>
#include <csapex_point_cloud/math/mean.hpp>
#include <pcl/point_types.h>

namespace csapex
{
namespace clustering
{
class ColorFeature
{
public:
    inline void create(...)
    {
    }

    inline void create(const pcl::PointXYZI& point)
    {
        math::Mean<1> mean;
        mean.add(point.intensity);
        color = std::move(mean);
    }

    inline void create(const pcl::PointXYZRGB& point)
    {
        math::Mean<3> mean;
        mean.add({ static_cast<double>(point.r), static_cast<double>(point.g), static_cast<double>(point.b) });
        color = std::move(mean);
    }

    inline void merge(const ColorFeature& other)
    {
        static UpdateMean updater;
        boost::apply_visitor(updater, color, other.color);
    }

    using DifferenceFunction = double (*)(const Eigen::Vector3d&, const Eigen::Vector3d&, const std::array<double, 3>&);

    inline double difference(const DifferenceFunction& difference_fn, const std::array<double, 3>& weights, const ColorFeature& other) const
    {
        return boost::apply_visitor(Difference(difference_fn, weights), color, other.color);
    }

private:
    struct UpdateMean : boost::static_visitor<void>
    {
        template <std::size_t N>
        void operator()(math::Mean<N>& self, const math::Mean<N>& other) const
        {
            self += other;
        }

        template <std::size_t N1, std::size_t N2, typename = typename std::enable_if<N1 != N2>::type>
        void operator()(math::Mean<N1>& self, const math::Mean<N2>& other) const
        {
        }
    };

    struct Difference : boost::static_visitor<double>
    {
        Difference(const DifferenceFunction& difference_fn, const std::array<double, 3>& weights) : difference_fn_(difference_fn), weights(weights)
        {
        }

        /// greyscale case, use direct difference
        double operator()(const math::Mean<1>& self, const math::Mean<1>& other) const
        {
            return std::abs(self.getMean() - other.getMean());
        }

        /// color case, use color difference function
        double operator()(const math::Mean<3>& self, const math::Mean<3>& other) const
        {
            return difference_fn_(self.getMean(), other.getMean(), weights);
        }

        template <std::size_t N1, std::size_t N2, typename = typename std::enable_if<N1 != N2>::type>
        double operator()(const math::Mean<N1>& self, const math::Mean<N2>& other) const
        {
            return 0.0;
        }

    private:
        const DifferenceFunction& difference_fn_;
        const std::array<double, 3>& weights;
    };

private:
    boost::variant<math::Mean<1>, math::Mean<3>> color;
};

}  // namespace clustering
}  // namespace csapex
