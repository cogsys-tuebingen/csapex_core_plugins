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

    virtual void process() override;
    virtual void setup(csapex::NodeModifier& node_modifier) override;
    virtual void setupParameters(Parameterizable& parameters);

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
