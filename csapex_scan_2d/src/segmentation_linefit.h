#ifndef SEGMNETATION_LINEFIT_H
#define SEGMNETATION_LINEFIT_H

#include "segmentation.h"

namespace csapex {

class LineFitSegmentation : public ScanSegmentation
{
public:
    LineFitSegmentation();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void update();
};

}
#endif // SEGMNETATION_LINEFIT_H
