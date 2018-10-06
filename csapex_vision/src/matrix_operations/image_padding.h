#ifndef IMAGE_PADDING_H
#define IMAGE_PADDING_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class ImagePadding : public Node
{
public:
    ImagePadding();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters);
    virtual void process() override;

private:
    Input* input_;
    Output* output_;
    Output* output_mask_;
};

}  // namespace csapex

#endif  // IMAGE_PADDING_H
