/// HEADER
#include <csapex_math/model/vector.h>

using namespace csapex;
using namespace csapex::math;
using namespace csapex::math::linear;

Vector::Vector(std::vector<double> value)
    : data_(std::move(value))
{
}

Vector::Vector(std::initializer_list<double> value)
    : data_{value}
{
}


double Vector::operator() (int row) const
{
    return operator[](row);
}
double& Vector::operator() (int row)
{
    return operator[](row);
}
double Vector::operator[] (int row) const
{
    return data_[row];
}
double& Vector::operator[] (int row)
{
    return data_[row];
}


int Vector::rows() const
{
    return data_.size();
}
int Vector::cols() const
{
    return 1;
}

const std::vector<double>& Vector::getDataRef() const
{
    return data_;
}
std::vector<double> Vector::getData() const
{
    return data_;
}
std::size_t Vector::size() const
{
    return data_.size();
}

bool Vector::isEqual(const Vector& rhs) const
{
    return data_ == rhs.data_;
}
