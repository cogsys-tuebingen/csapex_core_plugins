#ifndef DIFFERENCE_MAXIMUM_H
#define DIFFERENCE_MAXIMUM_H

/// COMPONENT
#include "corner_line_detection.h"

namespace csapex
{
class DifferenceMaximum : public CornerLineDetection
{
public:
    DifferenceMaximum();

    void process() override;
    void setupParameters(Parameterizable& parameters) override;
};
}  // namespace csapex
#endif  // DIFFERENCE_MAXIMUM_H
