#ifndef IMAGE_PADDING_H
#define IMAGE_PADDING_H

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM
#include <QSlider>

namespace csapex
{

class ImagePadding : public Node
{
public:
    ImagePadding();

    void setup();
    void process();

private:
    Input* input_;
    Output* output_;
    Output* output_mask_;
};

} /// NAMESPACE

#endif // IMAGE_PADDING_H
