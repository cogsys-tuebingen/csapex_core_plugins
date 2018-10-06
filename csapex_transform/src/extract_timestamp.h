#ifndef EXTRACT_TIMESTAMP_H_
#define EXTRACT_TIMESTAMP_H_

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class ExtractTimeStamp : public Node
{
public:
    ExtractTimeStamp();

    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void process() override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // EXTRACT_TIMESTAMP_H_
