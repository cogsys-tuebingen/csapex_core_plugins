#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class MedianFilter : public csapex::Node
{
public:
    MedianFilter();

    virtual void process();
    virtual void setup();

private:
    int           kernel_size_;
    ConnectorOut* output_;
    ConnectorIn*  input_;

    void update();
};
}
#endif // MEDIAN_FILTER_H
