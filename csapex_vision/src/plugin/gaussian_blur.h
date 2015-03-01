#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class GaussianBlur : public Node
{
public:
    GaussianBlur();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    void update();

    Input  *input_;
    Output *output_;

    int           kernel_;
    double        sigma_x_;
    double        sigma_y_;
};

} /// NAMESPACE

#endif // FilterBlur_H
