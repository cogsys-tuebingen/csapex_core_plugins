/// HEADER
#include <csapex_point_cloud/msg/binary_io.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

/// SERIALIZATION
SerializationBuffer& operator << (SerializationBuffer& data, const pcl::PCLHeader& header)
{
    data << header.frame_id;
    data << header.seq;
    data << header.stamp;
    return data;
}
const SerializationBuffer& operator >> (const SerializationBuffer& data, pcl::PCLHeader& header)
{
    data >> header.frame_id;
    data >> header.seq;
    data >> header.stamp;
    return data;
}

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const pcl::PointIndices& pi)
{
    data << pi.header;
    data << pi.indices;
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, pcl::PointIndices& pi)
{
    data >> pi.header;
    data >> pi.indices;
    return data;
}


SerializationBuffer& csapex::operator << (SerializationBuffer& data, const pcl::ModelCoefficients& ci)
{
    data << ci.header;
    data << ci.values;
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, pcl::ModelCoefficients& ci)
{
    data >> ci.header;
    data >> ci.values;
    return data;
}
