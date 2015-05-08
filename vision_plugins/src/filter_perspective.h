#ifndef FILTER_PERSPECTIVE_H
#define FILTER_PERSPECTIVE_H

/// COMPONENT
#include <csapex_vision/filter.h>
#include <utils_vision/utils/perspective_transform.h>

namespace vision_plugins {

class PerspectiveTransform : public csapex::Filter
{
public:
    PerspectiveTransform();

    virtual void filter(cv::Mat &img, cv::Mat &mask) override;
    virtual void setupParameters(Parameterizable &parameters) override;

    void update();

private:
    utils_vision::PerspectiveTransformer transformer_;
};

}
#endif // FILTER_PERSPECTIVE_H
