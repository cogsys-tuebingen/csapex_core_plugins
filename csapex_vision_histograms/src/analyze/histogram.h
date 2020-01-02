#ifndef HISTOGRAM_VISION_H
#define HISTOGRAM_VISION_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class Histogram : public csapex::Node
{
public:
    Histogram();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* output_;
    csapex::Input* input_;
    csapex::Input* mask_;

    int bins_;
    int last_type_;
    bool uniform_;
    bool accumulate_;
    bool min_max_;
    bool min_max_global_;
    bool append_;
    std::pair<float, float> min_max_value_;

    void update();

    void resetMinMax();
};
}  // namespace csapex

#endif  // HISTOGRAM_VISION_H
