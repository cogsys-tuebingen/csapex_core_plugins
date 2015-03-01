#ifndef IMAGE_STITCHER_H
#define IMAGE_STITCHER_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class MatrixStitcher : public csapex::Node
{
public:
    MatrixStitcher();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    enum Mode {HORIZONTAL, VERTICAL};

    csapex::Input     *matrix_1_;
    csapex::Input     *matrix_2_;
    csapex::Output    *stitched_;
};
}
#endif // IMAGE_STITCHER_H
