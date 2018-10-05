#ifndef SEQUENCE_MEAN_H
#define SEQUENCE_MEAN_H

/// COMPONENT
#include <csapex/model/node.h>
#include <deque>
#include <opencv2/core/core.hpp>

namespace csapex
{
class SequenceMean : public csapex::Node
{
public:
    SequenceMean();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Input* input_;
    csapex::Output* output_;

    int type_;
    int channels_;
    cv::Size size_;
    std::deque<cv::Mat> acc_;

    unsigned int sequence_size_;

    void update();
    bool check(const cv::Mat& mat);
};
}  // namespace csapex

#endif  // SEQUENCE_MEAN_H
