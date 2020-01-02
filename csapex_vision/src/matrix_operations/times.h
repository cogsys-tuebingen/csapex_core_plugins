#ifndef TIMES_H
#define TIMES_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class Times : public csapex::Node
{
public:
    Times();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

protected:
    csapex::Output* output_;
    csapex::Input* input_a_;
    csapex::Input* input_b_;

    double scale_;
    int dtype_;
};
}  // namespace csapex

#endif  // TIMES_H
