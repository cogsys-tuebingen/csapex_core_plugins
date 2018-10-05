#ifndef VECTOR_H
#define VECTOR_H

/// COMPONENT
#include <csapex_math/csapex_math_export.h>
#include <csapex_math/model/matrix.h>

/// SYSTEM
#include <iosfwd>
#include <vector>

namespace csapex
{
namespace math
{
namespace linear
{
class CSAPEX_MATH_EXPORT Vector
{
    friend std::ostream& operator<<(std::ostream& stream, const Vector& v);

public:
    Vector() = default;
    Vector(std::vector<double> value);
    Vector(std::initializer_list<double> value);

    double operator()(int row) const;
    double& operator()(int row);
    double operator[](int row) const;
    double& operator[](int row);

    bool isEqual(const Vector& rhs) const;

    friend bool operator==(const Vector& lhs, const Vector& rhs)
    {
        return lhs.isEqual(rhs);
    }

    friend bool operator!=(const Vector& lhs, const Vector& rhs)
    {
        return !(lhs.isEqual(rhs));
    }

    int rows() const;
    int cols() const;

    std::size_t size() const;
    void resize(std::size_t new_size, double value = 0.0);

    const std::vector<double>& getDataRef() const;
    std::vector<double> getData() const;

private:
    std::vector<double> data_;
};

}  // namespace linear
}  // namespace math
}  // namespace csapex

#endif  // VECTOR_H
