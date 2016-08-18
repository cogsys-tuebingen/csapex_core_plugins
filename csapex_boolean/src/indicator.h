#ifndef INDICATOR_H
#define INDICATOR_H

/// HEADER
#include <csapex/model/node.h>

namespace csapex {

namespace boolean {

class CSAPEX_EXPORT_PLUGIN Indicator : public Node
{
public:
    Indicator();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

    virtual void process() override;

private:
    Input* in;
};

}

}

#endif // INDICATOR_H
