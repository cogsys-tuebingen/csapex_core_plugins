#ifndef SEGMENTATIONSEGMENTLENGTH_H
#define SEGMENTATIONSEGMENTLENGTH_H

#include "segmentation.h"

namespace csapex {

class SegmentLengthSegmentation : public ScanSegmentation
{
public:
    SegmentLengthSegmentation();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void update();
};

}
#endif // SEGMENTATIONSEGMENTLENGTH_H
