#ifndef LAPLACIAN_H
#define LAPLACIAN_H

/// COMPONENT
#include "operator.h"

namespace csapex
{
class Laplacian : public Operator
{
public:
    Laplacian();

    virtual void process() override;
};
}  // namespace csapex
#endif  // LAPLACIAN_H
