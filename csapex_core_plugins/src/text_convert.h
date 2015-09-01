#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QLabel>

namespace csapex {

class TextConvert : public Node
{
public:
    TextConvert();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

private:
    Input* input_;
    Output* output_;
};

}

#endif // TEXT_DISPLAY_H_
