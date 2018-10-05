#ifndef TYPE_CONVERTER_H
#define TYPE_CONVERTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class ConvertType : public csapex::Node
{
public:
    ConvertType();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Input* input_;
    csapex::Output* output_;

    void update();

    int mode_;
    bool normalize_;
};
}  // namespace csapex
#endif  // TYPE_CONVERTER_H
