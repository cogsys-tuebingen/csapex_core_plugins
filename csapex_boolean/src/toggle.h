#ifndef TOGGLE_H
#define TOGGLE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
namespace boolean
{
class CSAPEX_EXPORT_PLUGIN Toggle : public Node
{
public:
    Toggle();

public:
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    void setSignal();

private:
    Output* out_;
    Event* event_;

    bool signal_;
};

}  // namespace boolean

}  // namespace csapex

#endif  // TOGGLE_H
