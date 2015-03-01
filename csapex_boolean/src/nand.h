#ifndef NAND_H
#define NAND_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

namespace boolean {

class NAND : public Node
{
public:
    NAND();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* in_a;
    Input* in_b;
    Output* out;
};

}

}

#endif // NAND_H
