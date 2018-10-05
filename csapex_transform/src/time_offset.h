#ifndef TIME_OFFSET_H_
#define TIME_OFFSET_H_

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class TimeOffset : public csapex::Node
{
public:
    TimeOffset();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

private:
    Output* output_;
    Input* input_;
};

}  // namespace csapex

#endif  // TIME_OFFSET_H_
