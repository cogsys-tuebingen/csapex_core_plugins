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
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

    void reset();

private:
    void update(const csapex::param::Parameter *);

private:
    Input* in_image_;
    Input* in_keypoints_;

    Output* out_debug_;

    bool init_;

    std::vector<cv::Point2f> points[2];

    cv::Mat gray;
    cv::Mat prevGray;
};

}

#endif // FILTER_TOOL_DETECTION_H
