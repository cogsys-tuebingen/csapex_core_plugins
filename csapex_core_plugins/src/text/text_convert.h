#ifndef TEXT_DISPLAY_H_
#define TEXT_DISPLAY_H_

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QLabel>

namespace csapex
{
class CSAPEX_EXPORT_PLUGIN TextConvert : public Node
{
public:
    TextConvert();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // TEXT_DISPLAY_H_
