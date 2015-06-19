#ifndef FILTER_DEBAYER_H
#define FILTER_DEBAYER_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>
class QComboBox;

namespace vision_plugins {
/**
 * @brief The Debayer class can be used to debayer raw images to get color images.
 */
class Debayer : public csapex::Node
{
public:
    Debayer();

    void setup(csapex::NodeModifier& node_modifier) ;
    void setupParameters(Parameterizable& parameters);

    /**
     * @brief See base class definition.
     */
    virtual void process();

private:
    virtual bool usesMask();
    void debayerAndResize(cv::Mat& source, cv::Mat& dest);
    csapex::Input* input_;
    csapex::Output* output_;

};

}
#endif // FILTER_DEBAYER_H
