#ifndef SOBEL_H
#define SOBEL_H

/// COMPONENT
#include "operator.h"

namespace vision_plugins {
class Sobel : public Operator
{
public:
    Sobel();

    virtual void process();
    virtual void setupParameters();

protected:
    int     dx_;
    int     dy_;

    void update();

};
}
#endif // SOBEL_H
