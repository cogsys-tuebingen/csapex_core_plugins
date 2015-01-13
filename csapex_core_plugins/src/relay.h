#ifndef RELAY_H
#define RELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

namespace csapex {

class Relay : public Node
{
public:
    Relay();

    virtual void setup();
    virtual void process();

private:
    Input* input_;
    Output* output_;
};

}

#endif // RELAY_H
