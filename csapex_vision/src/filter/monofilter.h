#ifndef MONOFILTER_H
#define MONOFILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class MonoFilter : public csapex::Node
{
public:
    MonoFilter();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;

    int min_;
    int max_;
    int def_;
    bool invert_;

    void update();
};

}  // namespace csapex

#endif  // MONOFILTER_H
