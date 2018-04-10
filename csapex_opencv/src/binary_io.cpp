/// HEADER
#include <csapex_opencv/binary_io.h>

/// PROJECT
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const cv::Mat& mat)
{
    const uint32_t elem_size = mat.elemSize();
    const uint32_t elem_type = mat.type();

    data << mat.cols;
    data << mat.rows;
    data << elem_size;
    data << elem_type;

    const uint32_t data_size = mat.cols * mat.rows * elem_size;
    data.writeRaw(mat.ptr(), data_size);

    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, cv::Mat& mat)
{
    uint32_t elem_size;
    uint32_t elem_type;

    int rows, cols;

    data >> cols;
    data >> rows;
    data >> elem_size;
    data >> elem_type;

    mat.create(rows, cols, elem_type);

    const uint32_t data_size = mat.cols * mat.rows * elem_size;
    data.readRaw(mat.ptr(), data_size);

    return data;
}

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const cv::KeyPoint& kp)
{
    data << kp.pt;
    data << kp.size;
    data << kp.angle;
    data << kp.response;
    data << kp.octave;
    data << kp.class_id;
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, cv::KeyPoint& kp)
{
    data >> kp.pt;
    data >> kp.size;
    data >> kp.angle;
    data >> kp.response;
    data >> kp.octave;
    data >> kp.class_id;
    return data;
}

SerializationBuffer& csapex::operator << (SerializationBuffer& data, const cv::DMatch& match)
{
    data << match.queryIdx;
    data << match.trainIdx;
    data << match.imgIdx;
    data << match.distance;
    return data;
}
const SerializationBuffer& csapex::operator >> (const SerializationBuffer& data, cv::DMatch& match)
{
    data >> match.queryIdx;
    data >> match.trainIdx;
    data >> match.imgIdx;
    data >> match.distance;
    return data;
}
