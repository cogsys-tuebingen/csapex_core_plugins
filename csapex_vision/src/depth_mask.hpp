#pragma once

#include <csapex/model/node.h>
#include <opencv2/opencv.hpp>

namespace csapex
{
namespace vision
{
class DepthMask : public csapex::Node
{
public:
    enum class Method
    {
        MEDIAN,
        MEAN,
        HISTOGRAM
    };

    void setupParameters(csapex::Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    void processRegion(const cv::Mat& image, cv::Mat mask);

private:
    Input* in_image_;
    Input* in_rois_;
    Output* out_mask_;

    Method method_;
    double threshold_;
    bool invert_;

    int offset_x_;
    int offset_y_;
};

}  // namespace vision
}  // namespace csapex
