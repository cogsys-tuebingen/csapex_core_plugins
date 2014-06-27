#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

/// COMPONENT
#include <csapex/model/node.h>

namespace vision_plugins {
class MedianFilter : public csapex::Node
{
public:
    MedianFilter();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

private:
    int           kernel_size_;
    csapex::ConnectorOut* output_;
    csapex::ConnectorIn*  input_;

    void update();
};
}
#endif // MEDIAN_FILTER_H
