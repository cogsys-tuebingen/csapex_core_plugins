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

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    void update();

    Input* input_mat_;
    Input* input_mask_;
    Output* output_;

    int kernel_;
    double sigma_x_;
    double sigma_y_;
};

}  // namespace csapex

#endif  // FilterBlur_H
