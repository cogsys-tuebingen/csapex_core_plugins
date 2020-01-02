#ifndef GLCM_H
#define GLCM_H
/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class GLCM : public csapex::Node
{
public:
    GLCM();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Input* in_;
    csapex::Output* out_;
};
}  // namespace csapex

#endif  // GLCM_H
