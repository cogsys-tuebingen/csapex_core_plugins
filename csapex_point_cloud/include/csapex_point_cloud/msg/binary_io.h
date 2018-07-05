#ifndef PCL_BINARY_IO_H
#define PCL_BINARY_IO_H
/// PROJECT
#include <csapex/serialization/serialization_fwd.h>

/// FORWARD DECLARATION
namespace pcl
{
class PCLHeader;
class PointIndices;
class ModelCoefficients;
}

namespace csapex
{

SerializationBuffer& operator << (SerializationBuffer& data, const pcl::PCLHeader& header);
const SerializationBuffer& operator >> (const SerializationBuffer& data, pcl::PCLHeader& header);

SerializationBuffer& operator << (SerializationBuffer& data, const pcl::PointIndices& pi);
const SerializationBuffer& operator >> (const SerializationBuffer& data, pcl::PointIndices& pi);

SerializationBuffer& operator << (SerializationBuffer& data, const pcl::ModelCoefficients& ci);
const SerializationBuffer& operator >> (const SerializationBuffer& data, pcl::ModelCoefficients& ci);

}
#endif // PCL_BINARY_IO_H
