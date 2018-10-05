#ifndef COL_H
#define COL_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class Col : public Node
{
public:
    Col();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

private:
    Input* input_;
    Output* output_;

    bool request_center_;
    void requestCenter();
};
}  // namespace csapex

#endif  // COL_H
