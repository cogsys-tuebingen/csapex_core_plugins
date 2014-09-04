#ifndef ROTATE_IMAGE_H
#define ROTATE_IMAGE_H

/// COMPONENT

/// PROJECT
#include <csapex/model/node.h>

/// SYSTEM

namespace csapex {


class RotateImage : public csapex::Node
{
public:
    RotateImage();

    void setupParameters();
    void setup();
    void process();

private:
    Input* in_;
    Output* out_;
};


}

#endif // ROTATE_IMAGE_H
