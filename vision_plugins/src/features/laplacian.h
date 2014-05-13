#ifndef LAPLACIAN_H
#define LAPLACIAN_H

/// COMPONENT
#include "operator.h"

namespace vision_plugins {
class Laplacian : public Operator
{
public:
    Laplacian();

    virtual void process();
};
}
#endif // LAPLACIAN_H
