#ifndef AND_H
#define AND_H


/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
namespace boolean
{
class CSAPEX_EXPORT_PLUGIN AND : public Node
{
public:
    AND();

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
#endif // AND_H
