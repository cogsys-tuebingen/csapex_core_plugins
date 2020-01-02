#ifndef LSD_H
#define LSD_H

/// COMPONENT
#include "corner_line_detection.h"

#define FIELD_LENGTH 160

namespace csapex
{
class LineSegmentDetector : public CornerLineDetection
{
public:
    /** Structure to store one argument definition and read value.
     */
    struct argument
    {
        char name[FIELD_LENGTH]; /* name to internally identify the argument */
        char desc[FIELD_LENGTH]; /* description */
        char id;                 /* letter used with '-' to use the option */
        char type;               /* i=int, d=double, s=str, b=bool */
        int required;
        int assigned;
        int def_value;              /* true or false, a default value is assigned? */
        char d_value[FIELD_LENGTH]; /* default value */
        char s_value[FIELD_LENGTH]; /* string found, also the value if 'str' */
        int i_value;
        double f_value;
        int min_set; /* true or false, is minimal value set? */
        double min;
        int max_set; /* true or false, is maximal value set? */
        double max;
    };

    LineSegmentDetector();

    void process() override;

    void setup(csapex::NodeModifier& node_modifier) override;

    double* image;
    int X, Y;
    double* segs;
    int n;
    int dim = 7;
    int* region;
    int regX, regY;
    int i, j;

private:
    csapex::Input* input_;
};
}  // namespace csapex
#endif  // LSD_H
