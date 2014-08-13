#ifndef ADAPTIVE_THRESHOLD_H
#define ADAPTIVE_THRESHOLD_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class AdaptiveThreshold : public Node
{
public:
    AdaptiveThreshold();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    Output* output_;

    Input* input_;
};

}

#endif // ADAPTIVE_THRESHOLD_H
