#ifndef HOUGH_LINESP_H
#define HOUGH_LINESP_H

/// COMPONENT
#include "corner_line_detection.h"

namespace csapex
{
class HoughLinesP : public CornerLineDetection
{
public:
    HoughLinesP();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    void update();

    double rho_;
    double theta_;
    int threshold_;
    double min_line_length_;
    double max_line_gap_;

private:
    csapex::Output* output_vector_;
};
}  // namespace csapex
#endif  // HOUGH_LINESP_H
