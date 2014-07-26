#ifndef IMAGE_TO_POINT_CLOUD_H
#define IMAGE_TO_POINT_CLOUD_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex_point_cloud/point_cloud_message.h>

namespace cv
{
class Mat;
}

namespace csapex
{

class ImageToPointCloud : public Node
{
public:
    ImageToPointCloud();

    virtual void setup();
    virtual void process();

private:
    template <typename PointT>
    connection_types::PointCloudMessage::Ptr
    transform(const cv::Mat& range, const cv::Mat& intensity);

private:
    Input* input_depth_;
    Input* input_intensity_;
    Output* output_;
};

}

#endif // IMAGE_TO_POINT_CLOUD_H
