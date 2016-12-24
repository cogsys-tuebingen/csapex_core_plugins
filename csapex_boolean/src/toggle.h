#ifndef TOGGLE_H
#define TOGGLE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

namespace boolean {

class CSAPEX_EXPORT_PLUGIN Toggle : public Node
{
public:
    Toggle();

public:
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    void setSignal();

private:
    Output* out_;
    Event* event_;

    bool signal_;
};

}

}

#endif // TOGGLE_H
