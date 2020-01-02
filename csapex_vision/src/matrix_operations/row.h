#ifndef ROW_H
#define ROW_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Row : public Node
{
public:
    Row();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_;
    Output* output_;

    bool request_center_;
    void requestCenter();
};
}  // namespace csapex

#endif  // ROW_H
