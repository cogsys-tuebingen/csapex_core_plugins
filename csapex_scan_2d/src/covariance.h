#ifndef SCAN_COVARIANCE_H
#define SCAN_COVARIANCE_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class ScanCovariance : public csapex::Node
{
public:
    ScanCovariance();

    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;
    void process() override;

private:
    Input* input_;
    Output* output_;
};

}  // namespace csapex

#endif  // SCAN_COVARIANCE_H
