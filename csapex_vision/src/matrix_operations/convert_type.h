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

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    csapex::Input* input_;
    csapex::Output* output_;

    void update();

    int    mode_;
    bool   normalize_;
    double alpha_;
    double beta_;
};
}  // namespace csapex
#endif  // TYPE_CONVERTER_H
