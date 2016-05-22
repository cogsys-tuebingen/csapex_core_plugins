#ifndef SEGMENTATIONELLIPSOID_H
#define SEGMENTATIONELLIPSOID_H

#include "segmentation.h"

namespace csapex {


class SegmentationEllipsoid : public ScanSegmentation
{
public:
    SegmentationEllipsoid();

    void setup(NodeModifier &node_modifier) override;
    void setupParameters(Parameterizable &parameters) override;

protected:
    void update();
};

}
#endif // SEGMENTATIONELLIPSOID_H
