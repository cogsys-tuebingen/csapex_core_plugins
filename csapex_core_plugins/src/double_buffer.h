#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

namespace csapex {

class DoubleBuffer : public Node
{
public:
    DoubleBuffer();

    virtual void setup();
    virtual void process();
    virtual void tick();

private:
    void swapBuffers();

private:
    Input* input_;
    Output* output_;

    bool dirty_;

    ConnectionType::Ptr buffer_back_;
    ConnectionType::Ptr buffer_front_;
};

}

#endif // DOUBLE_BUFFER_H
