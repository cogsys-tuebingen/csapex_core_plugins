#ifndef SCHARR_H
#define SCHARR_H

/// COMPONENT
#include "operator.h"

namespace vision_plugins {
class Scharr : public Operator
{
public:
    Scharr();

    virtual void process();
    virtual void setupParameters();

protected:
    enum Type {DX1, DY1, STRENGTH};
    Type type_;

    void update();
};
}
#endif // SCHARR_H
