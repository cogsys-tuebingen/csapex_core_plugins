#ifndef BINARY_IO_H
#define BINARY_IO_H

/// COMPONENT
#include <csapex_math/model/matrix.h>
#include <csapex_math/model/vector.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

namespace csapex
{
SerializationBuffer& operator<<(SerializationBuffer& data, const math::linear::Vector& vector);
const SerializationBuffer& operator>>(const SerializationBuffer& data, math::linear::Vector& vector);

SerializationBuffer& operator<<(SerializationBuffer& data, const math::linear::Matrix& matrix);
const SerializationBuffer& operator>>(const SerializationBuffer& data, math::linear::Matrix& matrix);

}  // namespace csapex

#endif  // BINARY_IO_H
