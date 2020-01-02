#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class BoxBlur : public Node
{
public:
    BoxBlur();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // FilterBlur_H
