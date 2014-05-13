#ifndef COMBINER_SET_OPERATION_H
#define COMBINER_SET_OPERATION_H

/// PROJECT
#include <csapex/model/node.h>

namespace vision_plugins
{

/**
 * @brief The SetOperation class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the mean value of values given
 *        in a grid cell.
 */
class SetOperation : public csapex::Node
{
    enum {
        COMPLEMENT   = 0,
        INTERSECTION = 1,
        UNION        = 2
    };

public:
    SetOperation();

    void setup();
    void process();

private:
    csapex::ConnectorIn* i1_;
    csapex::ConnectorIn* i2_;
    csapex::ConnectorOut* out_;
};
}
#endif // COMBINER_SET_OPERATION_H
