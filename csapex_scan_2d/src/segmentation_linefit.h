#ifndef SEGMNETATION_LINEFIT_H
#define SEGMNETATION_LINEFIT_H

#include "segmentation.h"

namespace csapex
{
class LineFitSegmentation : public ScanSegmentation
{
public:
    LineFitSegmentation();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    void update();
};

}  // namespace csapex
#endif  // SEGMNETATION_LINEFIT_H
