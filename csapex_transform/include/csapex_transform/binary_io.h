#ifndef TF_BINARY_IO_H
#define TF_BINARY_IO_H

/// PROJECT
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <csapex/utility/suppress_warnings_start.h>
    #include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace csapex
{

SerializationBuffer& operator << (SerializationBuffer& data, const tf::Matrix3x3& mat);
const SerializationBuffer& operator >> (const SerializationBuffer& data, tf::Matrix3x3& mat);

SerializationBuffer& operator << (SerializationBuffer& data, const tf::Vector3& vec);
const SerializationBuffer& operator >> (const SerializationBuffer& data, tf::Vector3& vec);

SerializationBuffer& operator << (SerializationBuffer& data, const tf::Transform& tf);
const SerializationBuffer& operator >> (const SerializationBuffer& data, tf::Transform& tf);

}

#endif // TF_BINARY_IO_H
