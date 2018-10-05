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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Input* in_;
    csapex::Output* out_;
};
}  // namespace csapex

#endif  // GLCM_H
