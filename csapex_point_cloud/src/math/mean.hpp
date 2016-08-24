#ifndef MEAN_HPP
#define MEAN_HPP

#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace cspaex {
namespace math {

template<std::size_t Dim>
class Mean {
public:
    typedef std::shared_ptr<Mean<Dim>>      Ptr;

    typedef Eigen::Matrix<double, Dim, 1>   PointType;



};
}
}


#endif // MEAN_HPP
