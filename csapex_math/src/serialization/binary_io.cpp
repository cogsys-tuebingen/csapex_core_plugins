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
