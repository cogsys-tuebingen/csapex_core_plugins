#ifndef SCHARR_H
#define SCHARR_H

/// COMPONENT
#include "operator.h"

namespace csapex
{
class Scharr : public Operator
{
public:
    Scharr();

    void setupParameters(Parameterizable& parameters) override;
    void process() override;

protected:
    enum Type
    {
        DX1,
        DY1,
        STRENGTH
    };
    Type type_;

    void update() override;
};
}  // namespace csapex
#endif  // SCHARR_H
