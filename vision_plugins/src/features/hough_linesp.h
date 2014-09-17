#ifndef HOUGH_LINESP_H
#define HOUGH_LINESP_H

/// COMPONENT
#include "corner_line_detection.h"

namespace vision_plugins {
class HoughLinesP : public CornerLineDetection
{
public:
    HoughLinesP();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    void update();

    double rho_;
    double theta_;
    int    threshold_;
    double min_line_length_;
    double max_line_gap_;

private:
    csapex::Output* output_vector_;
};
}
#endif // HOUGH_LINESP_H
