#ifndef TOGGLE_H
#define TOGGLE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

namespace boolean {

class Toggle : public Node
{
public:
    Toggle();

public:
    virtual void setup();
    virtual void process();
    virtual void tick();

private:
    void setSignal();

private:
    Output* out;
    bool signal_;
};

}

}

#endif // TOGGLE_H
