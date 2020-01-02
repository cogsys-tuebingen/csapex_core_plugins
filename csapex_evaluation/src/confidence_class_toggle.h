#ifndef CONFIDENCE_CLASS_TOGGLE_H
#define CONFIDENCE_CLASS_TOGGLE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class ConfidenceClassToggle : public csapex::Node
{
public:
    ConfidenceClassToggle();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    Input* in_;
    Output* out_;
};

}  // namespace csapex

#endif  // CONFIDENCE_CLASS_TOGGLE_H
