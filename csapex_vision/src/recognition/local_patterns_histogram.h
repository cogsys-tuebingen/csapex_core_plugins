#ifndef LOCAL_PATTERNS_H
#define LOCAL_PATTERNS_H

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{
class LocalPatternsHistogram : public csapex::Node
{
public:
    LocalPatternsHistogram();

    void process() override;
    void setup(csapex::NodeModifier& node_modifier) override;
    void setupParameters(Parameterizable& parameters) override;

protected:
    enum Type
    {
        LBP,
        LTP
    };

    csapex::Input* in_img_;
    csapex::Input* in_rois_;
    csapex::Output* out_;
};
}  // namespace csapex
#endif  // LOCAL_PATTERNS_H
