#ifndef TEXT_INPUT_H
#define TEXT_INPUT_H

/// PROJECT
#include <csapex/model/tickable_node.h>

namespace csapex {

class CSAPEX_EXPORT_PLUGIN TextInput : public TickableNode
{
public:
    TextInput();

    virtual void tick() override;
    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;

protected:
    void publish();

private:
    Output* output_;
    Event* event_;

    std::string text_;
};

}

#endif // TEXT_INPUT_H
