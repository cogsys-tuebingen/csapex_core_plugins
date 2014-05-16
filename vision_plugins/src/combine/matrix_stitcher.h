#ifndef IMAGE_STITCHER_H
#define IMAGE_STITCHER_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class MatrixStitcher : public csapex::Node
{
public:
    MatrixStitcher();

    virtual void process();
    virtual void setup();

protected:
    enum Mode {HORIZONTAL, VERTICAL};

    csapex::ConnectorIn     *matrix_1_;
    csapex::ConnectorIn     *matrix_2_;
    csapex::ConnectorOut    *stitched_;
};
}
#endif // IMAGE_STITCHER_H
