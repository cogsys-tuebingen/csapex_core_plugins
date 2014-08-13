#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class BilateralFilter : public Node
{
public:
    BilateralFilter();

    virtual void process();
    virtual void setup();

protected:
    Input  *input_;
    Output *output_;

    int           d_;
    double        sigma_color_;
    double        sigma_space_;
};

} /// NAMESPACE

#endif // FilterBlur_H
