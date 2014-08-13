#ifndef VECTORIZE_PYRAMID_H
#define VECTORIZE_PYRAMID_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class VectorizePyramid : public Node
{
public:
    VectorizePyramid();

    virtual void process();
    virtual void setup();

private:
    Input  *input_;
    Output *output_;

};

}
#endif // VECTORIZE_PYRAMID_H
