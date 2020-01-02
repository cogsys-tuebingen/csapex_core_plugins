#ifndef HISTOGRAM_MAXIMA_H
#define HISTOGRAM_MAXIMA_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{
class HistogramMaxima : public csapex::Node
{
public:
    HistogramMaxima();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    csapex::Output* maxima_;
    csapex::Input* histograms_;
};
}  // namespace csapex
#endif  // HISTOGRAM_MAXIMA_H
