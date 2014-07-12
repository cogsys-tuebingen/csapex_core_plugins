#ifndef BINOMIAL_FILTER_H
#define BINOMIAL_FILTER_H

/// COMPONENT
#include <csapex/model/node.h>
#include <opencv2/core/core.hpp>

namespace vision_plugins {
class BinomialFilter : public csapex::Node
{
public:
    BinomialFilter();

    virtual void process();
    virtual void setup();
    virtual void setupParameters();

protected:
    int                     kernel_size_;
    cv::Mat                 kernel_;
    csapex::ConnectorOut*   output_;
    csapex::ConnectorIn*    input_;

};
}
#endif // BINOMIAL_FILTER_H
