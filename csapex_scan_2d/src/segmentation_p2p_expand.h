#ifndef SEGMENTATION_P2P_EXPAND_H
#define SEGMENTATION_P2P_EXPAND_H

#include "segmentation.h"

namespace csapex {

class P2PSegmentationExpand : public ScanSegmentation
{
public:
    P2PSegmentationExpand();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void update();
};

}
#endif // SEGMENTATION_P2P_EXPAND_H
