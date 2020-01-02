#ifndef SEGMENTATIONSEGMENTLENGTH_H
#define SEGMENTATIONSEGMENTLENGTH_H

#include "segmentation.h"

namespace csapex
{
class SegmentLengthSegmentation : public ScanSegmentation
{
public:
    SegmentLengthSegmentation();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    void update();
};

}  // namespace csapex
#endif  // SEGMENTATIONSEGMENTLENGTH_H
