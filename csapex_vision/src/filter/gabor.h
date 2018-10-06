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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

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
