#ifndef HARRIS_H
#define HARRIS_H

/// COMPONENT
#include "corner_line_detection.h"

namespace csapex
{
class CornerHarris : public CornerLineDetection
{
public:
    CornerHarris();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    void update();

    double k_;
    int k_size_;
    int block_size_;
    int border_type_;
};

}  // namespace csapex
#endif  // HARRIS_H
