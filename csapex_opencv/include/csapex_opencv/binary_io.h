#ifndef OPENCV_BINARY_IO_HPP
#define OPENCV_BINARY_IO_HPP

/// PROJECT
#include <csapex/serialization/serialization_fwd.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
SerializationBuffer& operator<<(SerializationBuffer& data, const cv::Mat& mat);
const SerializationBuffer& operator>>(const SerializationBuffer& data, cv::Mat& mat);

SerializationBuffer& operator<<(SerializationBuffer& data, const cv::KeyPoint& kp);
const SerializationBuffer& operator>>(const SerializationBuffer& data, cv::KeyPoint& kp);

SerializationBuffer& operator<<(SerializationBuffer& data, const cv::DMatch& kp);
const SerializationBuffer& operator>>(const SerializationBuffer& data, cv::DMatch& kp);

template <typename T>
SerializationBuffer& operator<<(SerializationBuffer& data, const cv::Point_<T>& p)
{
    data << p.x;
    data << p.y;
    return data;
}
template <typename T>
const SerializationBuffer& operator>>(const SerializationBuffer& data, cv::Point_<T>& p)
{
    data >> p.x;
    data >> p.y;
    return data;
}

template <typename T, int Dim>
SerializationBuffer& operator<<(SerializationBuffer& data, const cv::Vec<T, Dim>& p)
{
    for (int i = 0; i < Dim; ++i) {
        data << p[i];
    }
    return data;
}
template <typename T, int Dim>
const SerializationBuffer& operator>>(const SerializationBuffer& data, cv::Vec<T, Dim>& p)
{
    for (int i = 0; i < Dim; ++i) {
        data >> p[i];
    }
    return data;
}

}  // namespace csapex

#endif  // OPENCV_BINARY_IO_HPP
