#ifndef FLOAT_TO_UCHAR_H
#define FLOAT_TO_UCHAR_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class FloatToUchar : public csapex::Node
{
public:
    FloatToUchar();

    virtual void process();
    virtual void setup();

protected:
    enum Type {ABSOLUTE, RELATIVE, SCALE};

    Type           type_;

    ConnectorOut*  output_;
    ConnectorIn*   input_;

    void update();
};
}
#endif // FLOAT_TO_UCHAR_H
