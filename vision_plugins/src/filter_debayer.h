#ifndef FILTER_DEBAYER_H
#define FILTER_DEBAYER_H

/// COMPONENT
#include <csapex_vision/filter.h>

/// SYSTEM
class QComboBox;

namespace vision_plugins {
/**
 * @brief The Debayer class can be used to debayer raw images to get color images.
 */
class Debayer : public csapex::Filter
{
public:
    Debayer();

    void setupParameters();

    /**
     * @brief See base class definition.
     */
    virtual void filter(cv::Mat &img, cv::Mat &mask);

private:
    virtual bool usesMask();
    void debayerAndResize(cv::Mat& source, cv::Mat& dest);
};

}
#endif // FILTER_DEBAYER_H
