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

private:
    ConnectorOut* output_;

    ConnectorIn* input_;
};

}

#endif // ADAPTIVE_THRESHOLD_H
