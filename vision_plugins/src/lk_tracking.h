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
    virtual void process();
    virtual void setup();

    void reset();

private:
    void update(const param::Parameter *);

private:
    ConnectorIn* in_image_;
    ConnectorIn* in_keypoints_;

    ConnectorOut* out_debug_;

    bool init_;

    std::vector<cv::Point2f> points[2];

    cv::Mat gray;
    cv::Mat prevGray;
};

}

#endif // FILTER_TOOL_DETECTION_H
