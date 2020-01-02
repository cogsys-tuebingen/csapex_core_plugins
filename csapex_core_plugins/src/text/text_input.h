#ifndef TEXT_INPUT_H
#define TEXT_INPUT_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN TextInput : public Node
{
public:
    TextInput();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;
    Event* event_;

    std::string text_;
};

}  // namespace csapex

#endif  // TEXT_INPUT_H
