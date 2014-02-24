#ifndef FLIP_H
#define FLIP_H
/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class Flip : public csapex::Node
{
public:
    Flip();

    virtual void process();
    virtual void setup();

protected:
    ConnectorOut*                    output_;
    ConnectorIn*                     input_;

    int mode_;

    void update();
};
}

#endif // FLIP_H
