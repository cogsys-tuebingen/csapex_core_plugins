#ifndef VECTOR_H
#define VECTOR_H

/// COMPONENT
#include <csapex_math/csapex_math_export.h>

/// SYSTEM
#include <vector>
#include <iosfwd>

namespace csapex {
namespace math {
namespace linear {

class CSAPEX_MATH_EXPORT Vector
{
    friend std::ostream& operator << (std::ostream& stream, const Vector& v);

public:
    Vector() = default;
    Vector(std::vector<double> value);
    Vector(std::initializer_list<double> value);

    double operator() (int row) const;
    double& operator() (int row);
    double operator[] (int row) const;
    double& operator[] (int row);

    bool isEqual(const Vector& rhs) const;

    friend bool operator == (const Vector& lhs, const Vector& rhs)
    {
        return lhs.isEqual(rhs);
    }

    friend bool operator != (const Vector& lhs, const Vector& rhs)
    {
        return !(lhs.isEqual(rhs));
    }

    int rows() const;
    int cols() const;

    std::size_t size() const;

    const std::vector<double>& getDataRef() const;
    std::vector<double> getData() const;

private:
    std::vector<double> data_;
};

}
} // namespace math
} // namespace csapex

#endif // VECTOR_H
