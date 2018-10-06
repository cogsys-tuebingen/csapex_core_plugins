/// HEADER
#include <csapex_scan_2d/binary_io.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const lib_laser_processing::Scan::Header& header)
{
    data << header.seq;
    data << header.stamp_nsec;
    data << header.frame_id;
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, lib_laser_processing::Scan::Header& header)
{
    data >> header.seq;
    data >> header.stamp_nsec;
    data >> header.frame_id;
    return data;
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const lib_laser_processing::LaserBeam& beam)
{
    data << beam.yaw();
    data << beam.range();
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, lib_laser_processing::LaserBeam& beam)
{
    float yaw, range;
    data >> yaw;
    data >> range;
    beam = lib_laser_processing::LaserBeam(yaw, range);
    return data;
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const lib_laser_processing::Scan& scan)
{
    data << scan.header;
    data << scan.angle_min;
    data << scan.angle_max;
    data << scan.angle_increment;
    data << scan.range_min;
    data << scan.range_max;
    data << scan.rays;
    data << scan.valid;
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, lib_laser_processing::Scan& scan)
{
    data >> scan.header;
    data >> scan.angle_min;
    data >> scan.angle_max;
    data >> scan.angle_increment;
    data >> scan.range_min;
    data >> scan.range_max;
    data >> scan.rays;
    data >> scan.valid;
    return data;
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const lib_laser_processing::LabeledScan& scan)
{
    data << static_cast<const lib_laser_processing::Scan&>(scan);
    data << scan.labels;
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, lib_laser_processing::LabeledScan& scan)
{
    data >> static_cast<lib_laser_processing::Scan&>(scan);
    data >> scan.labels;
    return data;
}

SerializationBuffer& csapex::operator<<(SerializationBuffer& data, const lib_laser_processing::Segment& segment)
{
    data << segment.rays;
    data << segment.frame_id;
    data << segment.stamp_micro_seconds;
    data << segment.classification;
    data << segment.start_idx;
    return data;
}
const SerializationBuffer& csapex::operator>>(const SerializationBuffer& data, lib_laser_processing::Segment& segment)
{
    data >> segment.rays;
    data >> segment.frame_id;
    data >> segment.stamp_micro_seconds;
    data >> segment.classification;
    data >> segment.start_idx;
    return data;
}
