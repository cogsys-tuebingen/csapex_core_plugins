#ifndef SOBEL_H
#define SOBEL_H

/// COMPONENT
#include "operator.h"

namespace csapex {
class Sobel : public Operator
{
public:
    Sobel();

    virtual void process() override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    int     dx_;
    int     dy_;

    void update();

};
}
#endif // SOBEL_H
