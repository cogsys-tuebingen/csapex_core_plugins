#ifndef FILTER_TOOL_DETECTION_H
#define FILTER_TOOL_DETECTION_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
class LKTracking : public csapex::Node
{
public:
    LKTracking();

public:
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

    void reset() override;

private:
    void update(const csapex::param::Parameter*);

private:
    Input* in_image_;
    Input* in_keypoints_;

    Output* out_debug_;

    bool init_;

    std::vector<cv::Point2f> points[2];

    cv::Mat gray;
    cv::Mat prevGray;
};

}  // namespace csapex

#endif  // FILTER_TOOL_DETECTION_H
