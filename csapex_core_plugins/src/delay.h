#ifndef DELAY_H
#define DELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <QMutex>

namespace csapex {

class Delay : public Node
{
public:
    Delay();

    virtual void process();
    virtual void setup();

private:
    Input* input_;
    Output* output_;
};

}

#endif // DELAY_H
