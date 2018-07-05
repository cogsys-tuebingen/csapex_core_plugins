#ifndef SCAN2D_BINARY_IO_H
#define SCAN2D_BINARY_IO_H

/// PROJECT
#include <csapex/serialization/serialization_fwd.h>

/// PROJECT
#include <cslibs_laser_processing/data/labeled_scan.h>
#include <cslibs_laser_processing/data/scan.h>
#include <cslibs_laser_processing/data/segment.h>

/// SYSTEM
#include <csapex/utility/suppress_warnings_start.h>
    #include <tf/tf.h>
#include <csapex/utility/suppress_warnings_end.h>

namespace csapex
{
SerializationBuffer& operator << (SerializationBuffer& data, const lib_laser_processing::Scan::Header& header);
const SerializationBuffer& operator >> (const SerializationBuffer& data, lib_laser_processing::Scan::Header& header);

SerializationBuffer& operator << (SerializationBuffer& data, const lib_laser_processing::LaserBeam& beam);
const SerializationBuffer& operator >> (const SerializationBuffer& data, lib_laser_processing::LaserBeam& beam);

SerializationBuffer& operator << (SerializationBuffer& data, const lib_laser_processing::Scan& scan);
const SerializationBuffer& operator >> (const SerializationBuffer& data, lib_laser_processing::Scan& scan);

SerializationBuffer& operator << (SerializationBuffer& data, const lib_laser_processing::LabeledScan& scan);
const SerializationBuffer& operator >> (const SerializationBuffer& data, lib_laser_processing::LabeledScan& scan);

SerializationBuffer& operator << (SerializationBuffer& data, const lib_laser_processing::Segment& segment);
const SerializationBuffer& operator >> (const SerializationBuffer& data, lib_laser_processing::Segment& segment);

}

#endif // SCAN2D_BINARY_IO_H
