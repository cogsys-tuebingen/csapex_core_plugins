#ifndef SEGMENTATION_P2P_H
#define SEGMENTATION_P2P_H

#include "segmentation.h"

namespace csapex
{
class P2PSegmentation : public ScanSegmentation
{
public:
    P2PSegmentation();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    void update();
};

}  // namespace csapex
#endif  // SEGMENTATION_P2P_H
