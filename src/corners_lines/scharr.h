#ifndef SCHARR_H
#define SCHARR_H

/// COMPONENT
#include "operator.h"

namespace csapex {
class Scharr : public Operator
{
public:
    Scharr();

    virtual void process();

protected:
    enum Type {DX1, DY1};
    Type type_;

    void update();
};
}
#endif // SCHARR_H
