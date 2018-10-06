/// HEADER
#include <csapex_math/model/matrix.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <iostream>

using namespace csapex;
using namespace csapex::math;
using namespace csapex::math::linear;

Matrix::Matrix(const int rows, const int cols, std::vector<double> value) : rows_(rows), cols_(cols), data_(value)
{
    apex_assert_equal(rows * cols, static_cast<long>(data_.size()));
}

Matrix::Matrix(const int rows, const int cols, std::initializer_list<double> value) : rows_(rows), cols_(cols), data_{ value }
{
    apex_assert_equal(rows * cols, static_cast<long>(data_.size()));
}

double Matrix::operator()(const int row, const int col) const
{
    apex_assert_gte(row, 0);
    apex_assert_lt(row, rows_);

    apex_assert_gte(col, 0);
    apex_assert_lt(col, cols_);

    return data_.at(row * cols_ + col);
}
double& Matrix::operator()(const int row, const int col)
{
    return data_.at(row * cols_ + col);
}

int Matrix::rows() const
{
    return rows_;
}
int Matrix::cols() const
{
    return cols_;
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
void Matrix::resize(int rows, int cols, double value)
{
    // TODO: implement more efficiently
    int rows_old = rows_;
    int cols_old = cols_;
    auto cpy = data_;

    rows_ = rows;
    cols_ = cols;

    data_.resize(rows * cols, value);
    std::fill(data_.begin(), data_.end(), value);

    for (int row = 0, row_to = std::min(rows, rows_old); row < row_to; ++row) {
        for (int col = 0, col_to = std::min(cols, cols_old); col < col_to; ++col) {
            data_[row * cols + col] = cpy[row * cols_old + col];
        }
    }

}

bool Matrix::isEqual(const Matrix& rhs) const
{
    return data_ == rhs.data_;
}
