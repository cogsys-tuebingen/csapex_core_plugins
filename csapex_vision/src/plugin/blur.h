#ifndef FilterBlur_H
#define FilterBlur_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class BoxBlur : public Node
{
public:
    BoxBlur();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:

    Input  *input_;
    Output *output_;
};

} /// NAMESPACE

#endif // FilterBlur_H
