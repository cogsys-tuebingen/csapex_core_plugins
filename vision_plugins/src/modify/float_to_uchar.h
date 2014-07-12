#ifndef FLOAT_TO_UCHAR_H
#define FLOAT_TO_UCHAR_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class FloatToUchar : public csapex::Node
{
public:
    FloatToUchar();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    enum Type {ABSOLUTE, RELATIVE, SCALE};

    Type           type_;

    csapex::ConnectorIn*   input_;
    csapex::ConnectorOut*  output_;

    void update();
};
}
#endif // FLOAT_TO_UCHAR_H
