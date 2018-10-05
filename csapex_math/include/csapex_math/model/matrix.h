#ifndef MATRIX_H
#define MATRIX_H


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
class CSAPEX_MATH_EXPORT Matrix
{
    friend std::ostream& operator<<(std::ostream& stream, const Matrix& v);

public:
    Matrix() = default;
    Matrix(int rows, int cols, std::vector<double> value);
    Matrix(int rows, int cols, std::initializer_list<double> value);

    double operator()(int row) const;
    double& operator()(int row);
    double operator[](int row) const;
    double& operator[](int row);

    bool isEqual(const Matrix& rhs) const;

    friend bool operator==(const Matrix& lhs, const Matrix& rhs)
    {
        return lhs.isEqual(rhs);
    }

    friend bool operator!=(const Matrix& lhs, const Matrix& rhs)
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
    int rows_;
    int cols_;
    std::vector<double> data_;
};

}  // namespace linear
}  // namespace math
}  // namespace csapex

#endif  // MATRIX_H
