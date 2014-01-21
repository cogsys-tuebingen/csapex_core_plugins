#ifndef NUMBER_GENERATOR_H
#define NUMBER_GENERATOR_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class NumberGenerator : public csapex::Node
{
public:
    NumberGenerator();

    virtual void allConnectorsArrived();
    virtual void setup();

private:
    ConnectorIn* input_;
    ConnectorOut* output_;

    int n;
};
}

#endif // NUMBER_GENERATOR_H
