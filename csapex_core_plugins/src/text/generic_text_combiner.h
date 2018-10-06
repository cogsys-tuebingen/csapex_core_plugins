#ifndef GENERIC_IMAGE_COMBINER_H
#define GENERIC_IMAGE_COMBINER_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/variadic_io.h>
#include <csapex/utility/assert.h>

namespace csapex
{
class GenericTextCombiner : public csapex::Node, public csapex::VariadicInputs
{
public:
    GenericTextCombiner();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;
    virtual void process() override;

    virtual csapex::Input* createVariadicInput(csapex::TokenDataConstPtr type, const std::string& label, bool optional) override;

private:
    void updateFormula();

private:
    csapex::Output* out_;

    std::string format;
};

}  // namespace csapex

#endif  // GENERIC_IMAGE_COMBINER_H
