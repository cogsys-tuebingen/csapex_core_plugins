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

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

protected:
    Input  *input_;
    Output *output_;

    int           d_;
    double        sigma_color_;
    double        sigma_space_;
};

} /// NAMESPACE

#endif // FilterBlur_H
