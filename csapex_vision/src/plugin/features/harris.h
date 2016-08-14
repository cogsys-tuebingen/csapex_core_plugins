#ifndef HARRIS_H
#define HARRIS_H

/// COMPONENT
#include "corner_line_detection.h"

namespace vision_plugins {
class CornerHarris : public CornerLineDetection
{
public:
    CornerHarris();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    void update();

    double k_;
    int    k_size_;
    int    block_size_;
    int    border_type_;

};

}
#endif // HARRIS_H
