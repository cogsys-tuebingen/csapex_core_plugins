#ifndef EQUALIZE_H
#define EQUALIZE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {

class Equalize : public csapex::Node
{
public:
    Equalize();

    virtual void process();
    virtual void setup();

protected:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;
};

}

#endif // EQUALIZE_H
