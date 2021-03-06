#ifndef SEGMENTATION_P2PLINE_H
#define SEGMENTATION_P2PLINE_H

#include "segmentation.h"

namespace csapex
{
class P2PLineSegmentation : public ScanSegmentation
{
public:
    P2PLineSegmentation();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    void update();
};

}  // namespace csapex
#endif  // SEGMENTATION_P2PLINE_H
