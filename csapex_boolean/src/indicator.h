#ifndef INDICATOR_H
#define INDICATOR_H

/// HEADER
#include <csapex/model/node.h>

namespace csapex {

namespace boolean {

class Indicator : public Node
{
public:
    Indicator();

public:
    virtual void setup();

    virtual void process();

private:
    Input* in;
};

}

}

#endif // INDICATOR_H
