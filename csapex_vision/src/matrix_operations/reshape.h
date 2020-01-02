#ifndef ROW_H
#define ROW_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Reshape : public Node
{
public:
    Reshape();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Input* input_;
    Output* output_;
    unsigned int in_size_;
    unsigned int in_rows_;
    unsigned int in_cols_;
    unsigned int out_rows_;
    unsigned int out_cols_;

    void reset() override;
    bool reset_;
};
}  // namespace csapex

#endif  // ROW_H
