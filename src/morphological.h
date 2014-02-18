#ifndef MORPOLOCIAL_H
#define MORPOLOCIAL_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class Morpholocial : public csapex::Node
{
public:
    Morpholocial();

    virtual void process();
    virtual void setup();

private:
    ConnectorOut* output_;

    ConnectorIn* input_;
};
}

#endif // MORPOLOCIAL_H
