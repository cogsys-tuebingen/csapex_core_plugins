#ifndef FLIP_H
#define FLIP_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class Flip : public csapex::Node
{
public:
    Flip();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

protected:
    csapex::Output* output_;
    csapex::Input*  input_;

    int mode_;

};
}

#endif // FLIP_H
