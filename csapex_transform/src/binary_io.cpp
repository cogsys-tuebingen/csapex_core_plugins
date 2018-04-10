/// HEADER
#include <csapex_transform/binary_io.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>

using namespace csapex;


SerializationBuffer& csapex::operator << (SerializationBuffer& data, const tf::Matrix3x3& mat)
{
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            data << mat[row][col];
        }
    }
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, tf::Matrix3x3& mat)
{
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            data >> mat[row][col];
        }
    }
    return data;
}

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const tf::Vector3& vec)
{
    for(int row = 0; row < 3; ++row) {
        data << vec[row];
    }
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, tf::Vector3& vec)
{
    for(int row = 0; row < 3; ++row) {
        data >> vec[row];
    }
    return data;
}

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const tf::Transform& tf)
{
    data << tf.getOrigin();
    data << tf.getBasis();
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, tf::Transform& tf)
{
    tf::Vector3 origin;
    data >> origin;
    tf.setOrigin(origin);

    tf::Matrix3x3 basis;
    data >> basis;
    tf.setBasis(basis);

    return data;
}
