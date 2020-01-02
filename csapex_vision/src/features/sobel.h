#ifndef SOBEL_H
#define SOBEL_H

/// COMPONENT
#include "operator.h"

namespace csapex
{
class Sobel : public Operator
{
public:
    Sobel();

    void process() override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    int dx_;
    int dy_;

    void update() override;
};
}  // namespace csapex
#endif  // SOBEL_H
