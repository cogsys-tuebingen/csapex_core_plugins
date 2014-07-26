#ifndef TEXT_INPUT_H
#define TEXT_INPUT_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {

class TextInput : public Node
{
public:
    TextInput();

    void process();
    virtual void setup();

protected:
    void publish();

private:
    Output* connector_;
};

}

#endif // TEXT_INPUT_H
