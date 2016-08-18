#ifndef BINOMIAL_FILTER_H
#define BINOMIAL_FILTER_H

/// COMPONENT
#include <csapex/model/node.h>
#include <opencv2/core/core.hpp>

namespace csapex {
class BinomialFilter : public csapex::Node
{
public:
    BinomialFilter();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters) override;

protected:
    int                     kernel_size_;
    cv::Mat                 kernel_;
    csapex::Output*   output_;
    csapex::Input*    input_;

};
}
#endif // BINOMIAL_FILTER_H
