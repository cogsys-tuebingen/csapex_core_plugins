#ifndef SEGMNETATION_LINEFIT_LSQ_H
#define SEGMNETATION_LINEFIT_LSQ_H

#include "segmentation.h"

namespace csapex {

class LineFitSegmentationLSQ : public ScanSegmentation
{
public:
    LineFitSegmentationLSQ();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void update();
};

}
#endif // SEGMNETATION_LINEFIT_LSQ_H
