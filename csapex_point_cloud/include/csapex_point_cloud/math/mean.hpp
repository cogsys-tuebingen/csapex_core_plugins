#ifndef MEAN_HPP
#define MEAN_HPP

#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace csapex {
namespace math {
template<std::size_t Dim>
class Mean {
public:
    typedef std::shared_ptr<Mean<Dim>>      Ptr;
    typedef Eigen::Matrix<double, Dim, 1>   PointType;

    Mean() :
        mean(PointType::Zero()),
        n(1),
        n_1(0)
    {
    }

    inline void reset()
    {
        mean = PointType::Zero();
        n    = 1;
        n_1  = 0;
    }

    inline void add(const PointType &_p)
    {
        mean = (mean * n_1 + _p) / n;
        ++n;
        ++n_1;
    }

    inline Mean& operator+=(const PointType &_p)
    {
        add(_p);
        return *this;
    }

    inline Mean& operator+=(const Mean &other)
    {
        std::size_t _n = n_1 + other.n_1;
        PointType   _mean = (mean * n_1 + other.mean * other.n_1) / (double) _n;
        n   = _n + 1;
        n_1 = _n;
        mean = _mean;
        return *this;
    }

    inline std::size_t getN() const
    {
        return n_1;
    }

    inline PointType getMean() const
    {
        return mean;
    }

    inline void getMean(PointType &_mean) const
    {
        _mean = mean;
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    PointType   mean;
    std::size_t n;
    std::size_t n_1;
};

template<>
class Mean<1> {
public:
    typedef std::shared_ptr<Mean<1>>      Ptr;
    Mean() :
        mean(0.0),
        n(1),
        n_1(0)
    {
    }

    inline void reset()
    {
        mean = 0.0;
        n    = 1;
        n_1  = 0;
    }

    inline void add(const double &_p)
    {
        mean = (mean * n_1 + _p) / n;
        ++n;
        ++n_1;
    }

    inline Mean& operator+=(const double &_p)
    {
        add(_p);
        return *this;
    }

    inline Mean& operator+=(const Mean &other)
    {
        std::size_t _n = n_1 + other.n_1;
        double _mean = (mean * n_1 + other.mean * other.n_1) / (double) _n;
        n   = _n + 1;
        n_1 = _n;
        mean = _mean;
        return *this;
    }

    inline std::size_t getN() const
    {
        return n_1;
    }

    inline double getMean() const
    {
        return mean;
    }

    inline void getMean(double &_mean) const
    {
        _mean = mean;
    }

private:
    double      mean;
    std::size_t n;
    std::size_t n_1;
};
}
}


#endif // MEAN_HPP
