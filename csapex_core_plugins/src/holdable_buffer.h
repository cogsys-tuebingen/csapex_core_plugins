#ifndef HOLDABLE_BUFFER_H
#define HOLDABLE_BUFFER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

/// SYSTEM
#include <deque>

namespace csapex {
class HoldableBuffer : public Node
{
public:
    HoldableBuffer();

    virtual void setup();
    virtual void process();
    virtual void setupParameters();

private:
    Input  *in_;
    Output *out_;

    std::deque<ConnectionType::ConstPtr> buffer_;
};
}

#endif // BUFFER_H
