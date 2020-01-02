#ifndef INDICATOR_H
#define INDICATOR_H

/// HEADER
#include <csapex/model/node.h>

namespace csapex
{
namespace boolean
{
class CSAPEX_EXPORT_PLUGIN Indicator : public Node
{
public:
    Indicator();

public:
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

    void process() override;

private:
    Input* in;
};

}  // namespace boolean

}  // namespace csapex

#endif  // INDICATOR_H
