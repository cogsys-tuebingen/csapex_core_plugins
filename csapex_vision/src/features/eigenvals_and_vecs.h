#ifndef EIGENVALS_AND_VECS_H
#define EIGENVALS_AND_VECS_H

/// COMPONENT
#include "corner_line_detection.h"

namespace csapex
{
class EigenValsAndVecs : public CornerLineDetection
{
public:
    enum EigenType
    {
        MIN_EIGEN_VAL,
        EIGEN_VALS_AND_VECS
    };

    EigenValsAndVecs();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    void update();
    int k_size_;
    int block_size_;
    int border_type_;
    EigenType eigen_type_;
};
}  // namespace csapex
#endif  // EIGENVALS_AND_VECS_H
