#ifndef RELAY_H
#define RELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <QMutex>

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

    QMutex mutex_;
};

}

#endif // RELAY_H
