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
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

    virtual void process() override;
    void updateInputs();
    virtual void stateChanged() override;

private:
    Output *output_;

    uint64 stamp_;
    void collectMessage(std::vector<cv::Mat> &messages, Encoding &encoding);
};
}
#endif // FILTER_MERGER_H
