#ifndef FILTER_MERGER_H
#define FILTER_MERGER_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex_vision/encoding.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace csapex {

const static int MERGER_INPUT_MAX = 10;
/**
 * @brief The Merger class can be used to merge a certain amount of
 *        images.
 */
class Merger : public Node
{
public:
    /**
     * @brief Merger Constructor.
     */
    Merger();

    /**
     * @brief See base class documentation.
     */
    virtual void setup();

    void process();
    void updateInputs();
    void stateChanged();

private:
    Output *output_;

    void collectMessage(std::vector<cv::Mat> &messages, Encoding &encoding);
};
}
#endif // FILTER_MERGER_H
