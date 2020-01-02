#ifndef SCALE_H
#define SCALE_H

/// COMPONENT
#include <csapex/model/node.h>

/// SYSTEM
#include <opencv2/core/core.hpp>

namespace csapex
{
class Scale : public csapex::Node
{
public:
    Scale();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Output* output_;
    csapex::Input* input_;

    cv::Vec2d scales_;
    int mode_;
};

}  // namespace csapex
#endif  // SCALE_H
