#ifndef DELAY_H
#define DELAY_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/connection_type.h>

namespace csapex {

class Delay : public Node
{
public:
    Delay();

    virtual void setup();
    virtual void setupParameters();
    virtual void process();

private:
    Input* input_;
    Output* output_;

    param::OutputProgressParameter* progress_;
};

}

#endif // DELAY_H
