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
    ConnectorIn  *input_;
    ConnectorOut *output_;

};

}
#endif // VECTORIZE_PYRAMID_H
