#ifndef FILTER_DEBAYER_H
#define FILTER_DEBAYER_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex
{
/**
 * @brief The Debayer class can be used to debayer raw images to get color
 * images.
 */
class Debayer : public csapex::Node
{
public:
    Debayer();

    void setup(csapex::NodeModifier& node_modifier) override;

    void setupParameters(Parameterizable& parameters) override;

    /**
     * @brief See base class definition.
     */

    void process() override;

private:
    virtual bool usesMask();
    void debayerAndResize(const cv::Mat& source, cv::Mat& dest);

private:
    csapex::Input* input_img_;
    csapex::Output* output_img_;
};

}  // namespace csapex
#endif  // FILTER_DEBAYER_H
