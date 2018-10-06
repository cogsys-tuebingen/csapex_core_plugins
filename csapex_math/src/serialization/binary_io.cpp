/// HEADER
#include <csapex_math/serialization/binary_io.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const math::linear::Vector& vector)
{
    data << vector.getData();
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, math::linear::Vector& vector)
{
    std::vector<double> v;
    data >> v;
    vector = math::linear::Vector(v);
    return data;
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const math::linear::Matrix& matrix)
{
    data << static_cast<int32_t>(matrix.rows());
    data << static_cast<int32_t>(matrix.cols());
    data << matrix.getData();
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, math::linear::Matrix& vector)
{
    std::vector<double> v;
    int32_t rows, cols;
    data >> rows >> cols >> v;
    vector = math::linear::Matrix(rows, cols, v);
    return data;
}
