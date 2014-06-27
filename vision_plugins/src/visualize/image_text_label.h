#ifndef IMAGE_TEXT_LABEL_H
#define IMAGE_TEXT_LABEL_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins {
class ImageTextLabel : public csapex::Node
{
public:
    ImageTextLabel();

    void setup();
    void setupParameters();
    void process();

protected:
    enum Position {TOP_LEFT, BOTTOM_LEFT,
                   TOP_RIGHT, BOTTOM_RIGHT};

    csapex::ConnectorIn  *input_;
    csapex::ConnectorOut *output_;

};
}
#endif // IMAGE_TEXT_LABEL_H
