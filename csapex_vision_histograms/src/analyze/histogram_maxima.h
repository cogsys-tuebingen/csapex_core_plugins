#ifndef HISTOGRAM_MAXIMA_H
#define HISTOGRAM_MAXIMA_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex {
class HistogramMaxima : public csapex::Node
{
public:
    HistogramMaxima();

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

protected:
    csapex::Output* maxima_;
    csapex::Input*  histograms_;

};
}
#endif // HISTOGRAM_MAXIMA_H
