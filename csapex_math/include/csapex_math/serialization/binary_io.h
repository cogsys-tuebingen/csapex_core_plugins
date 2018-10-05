#ifndef BINARY_IO_H
#define BINARY_IO_H

/// COMPONENT
#include <csapex_math/model/vector.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

namespace csapex
{
SerializationBuffer& operator<<(SerializationBuffer& data, const math::linear::Vector& vector);
const SerializationBuffer& operator>>(const SerializationBuffer& data, math::linear::Vector& vector);

}  // namespace csapex

#endif  // BINARY_IO_H
