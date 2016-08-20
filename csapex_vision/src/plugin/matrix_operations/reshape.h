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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input  *input_;
    Output *output_;
    unsigned int in_size_;
    unsigned int in_rows_;
    unsigned int in_cols_;
    unsigned int out_rows_;
    unsigned int out_cols_;

    void reset();
    bool reset_;

};
}

#endif // ROW_H
