#ifndef HOUGH_CIRCLE_H
#define HOUGH_CIRCLE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

class HoughCircle : public Node
{
public:
    HoughCircle();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable &parameters) override;
    virtual void process() override;

private:
    Input* input_;
    Output* output_;
    Output* out_circles_;
};

}

#endif // HOUGH_CIRCLE_H
