#ifndef NAND_H
#define NAND_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
namespace boolean
{
class CSAPEX_EXPORT_PLUGIN NAND : public Node
{
public:
    NAND();

public:
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* in_a;
    Input* in_b;
    Output* out;
};

}  // namespace boolean

}  // namespace csapex

#endif  // NAND_H
