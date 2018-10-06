#ifndef PARAMETER_STREAM_IO_H
#define PARAMETER_STREAM_IO_H

/// COMPONENT
#include <csapex_math/model/vector.h>

/// SYSTEM
#include <ostream>

namespace csapex
{
namespace math
{
namespace linear
{
inline std::ostream& operator<<(std::ostream& stream, const Vector& vector)
{
    for (std::size_t row = 0; row < vector.size(); ++row) {
        if (row > 0) {
            stream << ", ";
        }
        stream << vector[row];
    }
    return stream;
}
inline std::ostream& operator<<(std::ostream& stream, const Matrix& matrix)
{
    int rows = matrix.rows();
    int cols = matrix.cols();
    for (int row = 0; row < rows; ++row) {
        if (row > 0) {
            stream << "\n";
        }
        for (int col = 0; col < cols; ++col) {
            stream << matrix(row, col) << " ";
        }
    }
    return stream;
}

}  // namespace linear
}  // namespace math
}  // namespace csapex

#endif  // PARAMETER_STREAM_IO_H
