#ifndef TEXT_INPUT_H
#define TEXT_INPUT_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TextInput : public Node
{
public:
    TextInput();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void publish();

private:
    Output* connector_;
};

}

#endif // TEXT_INPUT_H
