#ifndef IMAGE_TO_POINT_CLOUD_H
#define IMAGE_TO_POINT_CLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/msg/point_cloud_message.h>

namespace cv
{
class Mat;
}

namespace csapex
{

class ImageToPointCloud : public Node
{
public:
    enum class DepthType
    {
        METERS,
        MILLIMETERS,
        KINECT_TAN
    };

    enum class IntensityType
    {
        NONE,
        INTENSITY,
        RGB,
        BGR
    };

public:
    ImageToPointCloud();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    template <typename PointT, IntensityType>
    connection_types::PointCloudMessage::Ptr
    transform(const cv::Mat& range, const cv::Mat& intensity, std::uint64_t stamp);

    template <typename PointT, typename ImageType, IntensityType>
    connection_types::PointCloudMessage::Ptr
    transformImpl(const cv::Mat& range, const cv::Mat& intensity, std::uint64_t stamp);

    double depth_to_range(double) const;

private:
    Input* input_depth_;
    Input* input_intensity_;
    Output* output_;

    DepthType depth_type;
};

}

#endif // IMAGE_TO_POINT_CLOUD_H
