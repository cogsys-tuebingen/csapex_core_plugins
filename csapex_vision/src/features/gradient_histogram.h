#ifndef GRADIENTHISTOGRAM_H
#define GRADIENTHISTOGRAM_H

/// PROJECT
#include <csapex/model/node.h>

/// EXTRACT HOG FEATURE

namespace csapex {
class GradientHistogram : public csapex::Node
{
public:
    GradientHistogram();

    void setupParameters(Parameterizable& parameters) override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void process() override;

private:
    csapex::Input  *in_img_;
    csapex::Output *out_hist_;
    csapex::Output *out_mag_;

    std::array<double, 2> interval_;

    bool signed_;
    int  ksize_;

};
}
#endif // GRADIENTHISTOGRAM_H
