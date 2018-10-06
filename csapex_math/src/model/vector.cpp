/// HEADER
#include <csapex_math/model/vector.h>

using namespace csapex;
using namespace csapex::math;
using namespace csapex::math::linear;

Vector::Vector(std::vector<double> value) : Matrix(value.size(), 1, value)
{
}

Vector::Vector(std::initializer_list<double> value) : Matrix(value.size(), 1, { value })
{
}

double Vector::operator()(const int row) const
{
    return Matrix::operator()(row, 0);
}
double& Vector::operator()(const int row)
{
    return Matrix::operator()(row, 0);
}
double Vector::operator[](const int row) const
{
    return Matrix::operator()(row, 0);
}
double& Vector::operator[](const int row)
{
    return Matrix::operator()(row, 0);
}

bool Vector::isEqual(const Vector& rhs) const
{
    return data_ == rhs.data_;
}
