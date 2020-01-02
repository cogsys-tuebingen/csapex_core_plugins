#ifndef COMBINER_SET_OPERATION_H
#define COMBINER_SET_OPERATION_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
/**
 * @brief The SetOperation class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the mean value of values
 * given in a grid cell.
 */
class SetOperation : public csapex::Node
{
    enum
    {
        COMPLEMENT = 0,
        INTERSECTION = 1,
        UNION = 2
    };

public:
    SetOperation();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    csapex::Input* i1_;
    csapex::Input* i2_;
    csapex::Output* out_;
};
}  // namespace csapex
#endif  // COMBINER_SET_OPERATION_H
