#ifndef DIFFERENCE_MAXIMUM_H
#define DIFFERENCE_MAXIMUM_H

/// COMPONENT
#include "corner_line_detection.h"

namespace vision_plugins {
class DifferenceMaximum : public CornerLineDetection
{
public:
    DifferenceMaximum();

    virtual void process() override;
    virtual void setupParameters(Parameterizable& parameters);

};
}
#endif // DIFFERENCE_MAXIMUM_H
