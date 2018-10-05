/// HEADER
#include <csapex_math/model/matrix.h>


using namespace csapex;
using namespace csapex::math;
using namespace csapex::math::linear;

Matrix::Matrix(int rows, int cols, std::vector<double> value) : rows_(rows), cols_(cols), data_(std::move(value))
{
}

Matrix::Matrix(int rows, int cols, std::initializer_list<double> value) : rows_(rows), cols_(cols), data_{ value }
{
}

double Matrix::operator()(int row) const
{
    return operator[](row);
}
double& Matrix::operator()(int row)
{
    return operator[](row);
}
double Matrix::operator[](int row) const
{
    return data_[row];
}
double& Matrix::operator[](int row)
{
    return data_[row];
}

int Matrix::rows() const
{
    return data_.size();
}
int Matrix::cols() const
{
    return 1;
}

const std::vector<double>& Matrix::getDataRef() const
{
    return data_;
}
std::vector<double> Matrix::getData() const
{
    return data_;
}
std::size_t Matrix::size() const
{
    return data_.size();
}
void Matrix::resize(std::size_t new_size, double value)
{
    data_.resize(new_size, value);
}

bool Matrix::isEqual(const Matrix& rhs) const
{
    return data_ == rhs.data_;
}
