#ifndef OPERATOR_H
#define OPERATOR_H

#include "corner_line_detection.h"

namespace csapex
{
class Operator : public CornerLineDetection
{
public:
    Operator();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    int ddepth_;
    int ksize_;
    double scale_;
    double delta_;

    virtual void update();
};
}  // namespace csapex
#endif  // OPERATOR_H
