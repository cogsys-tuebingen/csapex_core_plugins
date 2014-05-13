#ifndef FILTER_PERSPECTIVE_H
#define FILTER_PERSPECTIVE_H

/// COMPONENT
#include <csapex_vision/filter.h>
#include <utils_cv/perspective_transform.h>

namespace vision_plugins {

class PerspectiveTransform : public csapex::Filter
{
public:
    PerspectiveTransform();

    virtual void filter(cv::Mat &img, cv::Mat &mask);

    void update();

private:
    utils_cv::PerspectiveTransformer transformer_;
};

}
#endif // FILTER_PERSPECTIVE_H
