#ifndef GABOR_H
#define GABOR_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class Gabor : public csapex::Node
{
public:
    Gabor();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;

    int ksize_;
    double sigma_;
    double theta_;
    double lambda_;
    double gamma_;
    double psi_;
};
}  // namespace csapex
#endif  // GABOR_H
