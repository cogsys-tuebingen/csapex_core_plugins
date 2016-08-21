#ifndef LABEL_REGIONS_H
#define LABEL_REGIONS_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

class LabelRegions : public csapex::Node
{
public:
    LabelRegions();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Output*   output_;
    csapex::Input*    input_;
};
}

#endif // LABEL_REGIONS_H
