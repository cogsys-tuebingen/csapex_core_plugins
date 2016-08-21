#ifndef IMAGE_TEXT_LABEL_H
#define IMAGE_TEXT_LABEL_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex {
class ImageTextLabel : public csapex::Node
{
public:
    ImageTextLabel();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

protected:
    enum Position {TOP_LEFT, BOTTOM_LEFT,
                   TOP_RIGHT, BOTTOM_RIGHT};

    csapex::Input  *input_;
    csapex::Output *output_;

};
}
#endif // IMAGE_TEXT_LABEL_H
