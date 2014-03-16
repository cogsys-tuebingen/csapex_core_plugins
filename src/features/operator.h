#ifndef OPERATOR_H
#define OPERATOR_H

#include "corner_line_detection.h"

namespace vision_plugins {
class Operator : public CornerLineDetection
{
public:
    Operator();

    virtual void setup();

protected:
    int     ddepth_;
    int     ksize_;
    double  scale_;
    double  delta_;

    virtual void update();
};
}
#endif // OPERATOR_H
