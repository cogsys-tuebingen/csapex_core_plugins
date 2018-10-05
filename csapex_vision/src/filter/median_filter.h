#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class MedianFilter : public csapex::Node
{
public:
    MedianFilter();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

private:
    int kernel_size_;
    csapex::Output* output_;
    csapex::Input* input_;

    void update();
};
}  // namespace csapex
#endif  // MEDIAN_FILTER_H
